#include "generalGPIO.h"

//-------- PRIVATE METHODS --------//

rcl_interfaces::msg::SetParametersResult GeneralGPIO::octetParameterCallback(
    const rclcpp::Parameter &parameter) {
    // Handle the octet parameter change
    this->debugLog("Octet parameter changed to " + std::to_string(parameter.as_int()));

    // Unsubscribe from the old response topic
    this->_response_subscription_.~Subscription();
    // Subscribe to the new response topic, at the new octet
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet" + std::to_string(parameter.as_int()) + "_response", 10,
        std::bind(&GeneralGPIO::responseCallback, this, std::placeholders::_1));

    // Publish a health message mentioning the octet change
    auto healthMsg = roi_ros::msg::Health();
    healthMsg.operation_status = 1;
    healthMsg.message = "Octet parameter changed to " + std::to_string(parameter.as_int());
    this->_health_publisher_->publish();

    // synchronize with the new module
    this->pushState();

    this->debugLog("Octet parameter change handled");

    return rcl_interfaces::msg::SetParametersResult();
}

rcl_interfaces::msg::SetParametersResult GeneralGPIO::aliasParameterCallback(
    const rclcpp::Parameter &parameter) {
    // Handle the alias parameter change
    this->debugLog("Alias parameter changed to " + parameter.as_string());

    // Publish a health message mentioning the alias change
    auto healthMsg = roi_ros::msg::Health();
    healthMsg.operation_status = 1;
    healthMsg.message = "Alias parameter changed to " + parameter.as_string();
    this->_health_publisher_->publish();

    this->debugLog("Alias parameter change handled");

    return rcl_interfaces::msg::SetParametersResult();
}

void GeneralGPIO::maintainState() {
    // Maintain the state of the GPIO module
    this->debugLog("Maintaining GPIO Module State");

    // Loop to maintain the state
    uint8_t checkResetCounter = 0;
    while (rclcpp::ok()) {
        // Check if the module has been reset, once every 128 loops (128*50ms = 6.4s)
        if (checkResetCounter == 0) {
            // Issues a status report request. The callback will handle the response.
            // If the callback receives a BLANKSTATE status, it will push the current state to the
            // module.
            ROIPackets::sysAdminPacket statusPacket = ROIPackets::sysAdminPacket();
            statusPacket.setMetaData(sysAdminConstants::NOCHAINMETA);
            statusPacket.setActionCode(sysAdminConstants::STATUSREPORT);

            this->sendSysadminPacket(statusPacket);
            checkResetCounter = 128;
        }
        checkResetCounter++;  // Increment the check reset counter

        // Loop through all the subdevices and read the inputs.
        for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
            if (this->subDeviceState[i] == GeneralGPIOConstants::INPUT_MODE ||
                this->subDeviceState[i] == GeneralGPIOConstants::INPUT_PULLUP_MODE) {
                this->sendReadPinPacket(i);
            }
        }

        // Sleep for 1 second
        std::this_thread::sleep_for(
            std::chrono::milliseconds(moduleNodeConstants::maintainStateSleepTime));
    }
}

void GeneralGPIO::responseCallback(const roi_ros::msg::SerializedPacket response) {
    // Handle the response from the transport agent
    this->debugLog("Received response from transport agent");

    // Parse the response packet
    ROIPackets::Packet packet = ROIPackets::Packet();
    uint8_t serializedData[ROIConstants::ROIMAXPACKETSIZE];
    this->unpackVectorToArray(response.data, serializedData, response.length);
    if (!packet.importPacket(serializedData, response.length) &&
        moduleNodeConstants::ignoreMalformedPackets) {
        this->debugLog("Failed to import packet");
        return;
    }

    // Handle the response packet
    uint8_t data[ROIConstants::ROIMAXPACKETPAYLOAD];
    packet.getData(data, ROIConstants::ROIMAXPACKETPAYLOAD);
    switch (packet.getActionCode()) {
        case GeneralGPIOConstants::READ_PIN:
            // Update local stores
            if (packet.getSubDeviceID() < 10) {
                this->subDeviceValue[packet.getSubDeviceID()] = data[0];  // Digital bool
            } else {
                subDeviceValue[packet.getSubDeviceID()] = data[0] << 8 | data[1];  // Analog int
            }

            // Publish the pin value
            this->publishPinValues();
            break;

        case GeneralGPIOConstants::SET_PIN_MODE:
            if (!data[0]) {
                this->debugLog("Failed to set pin mode for subdevice " +
                               std::to_string(packet.getSubDeviceID()));
            }
            // NONE todo. Set pinmode should update on outgoing request function.
            break;

        case GeneralGPIOConstants::SET_OUTPUT:
            if (!data[0]) {
                this->debugLog("Failed to set output for subdevice " +
                               std::to_string(packet.getSubDeviceID()));
            }
            // NONE todo. Set output should update on outgoing request function.
            break;

        default:
            this->debugLog("Unknown action code received: " +
                           std::to_string(packet.getActionCode()));
            break;
    }

    this->debugLog("Response handled");
}

void GeneralGPIO::publishPinStates() {
    // Publish the current pin states
    this->debugLog("Publishing Pin States");

    // Create the pin states message
    auto pinStatesMsg = roi_ros::msg::PinStates();
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        pinStatesMsg.pin_states.push_back(this->subDeviceState[i]);
    }

    // Publish the pin states message
    this->_pinStatesPublisher->publish(pinStatesMsg);

    this->debugLog("Pin States Published");
}

void GeneralGPIO::publishPinValues() {
    // Publish the current pin values
    this->debugLog("Publishing Pin Values");

    // Create the pin values message
    auto pinValuesMsg = roi_ros::msg::PinValues();
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        pinValuesMsg.pin_values.push_back(this->subDeviceValue[i]);
    }

    // Publish the pin values message
    this->_pinValuesPublisher->publish(pinValuesMsg);

    this->debugLog("Pin Values Published");
}

//-------- PUBLIC METHODS --------//

GeneralGPIO::GeneralGPIO() : BaseModule() {
    // Initialize the GPIO module
    this->debugLog("Initializing General GPIO Module");

    // Initialize the GPIO module parameters and callbacks
    this->_moduleAliasParameter = this->declare_parameter("module_alias", "blankGeneralGPIO");
    this->_moduleOctetParameter = this->declare_parameter("module_octet", 5);
    this->_octetParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&GeneralGPIO::octetParameterCallback, this, std::placeholders::_1));
    this->_aliasParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&GeneralGPIO::aliasParameterCallback, this, std::placeholders::_1));

    // Initialize the GPIO module health publisher
    this->_health_publisher_ = this->create_publisher<roi_ros::msg::Health>("health", 10);

    // Initialize the GPIO module general packet queue client
    this->_queue_general_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedGeneralPacket>("queue_general_packet");

    // Initialize the GPIO module response subscription
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet5_response", 10,
        std::bind(&GeneralGPIO::responseCallback, this, std::placeholders::_1));

    // Initialize the GPIO module specific publishers
    this->_pinStatesPublisher = this->create_publisher<roi_ros::msg::PinStates>("pin_states", 10);
    this->_pinValuesPublisher = this->create_publisher<roi_ros::msg::PinValues>("pin_values", 10);

    this->_setPinOutputService = this->create_service<roi_ros::srv::SetPinOutput>(
        "set_pin_output",
        std::bind(&GeneralGPIO::setPinOutput, this, std::placeholders::_1, std::placeholders::_2));
    this->_setPinStateService = this->create_service<roi_ros::srv::SetPinState>(
        "set_pin_state",
        std::bind(&GeneralGPIO::setPinState, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the GPIO module maintain state thread
    this->_maintainStateThread = std::thread(&GeneralGPIO::maintainState, this);
}

GeneralGPIO::~GeneralGPIO() {
    // Destroy the GPIO module
    this->debugLog("Destroying General GPIO Module");
    this->_maintainStateThread
        .join();  // Wait for the maintain state thread to finish (It should stop itself)
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeneralGPIO>());
    rclcpp::shutdown();
    return 0;
}