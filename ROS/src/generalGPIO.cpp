#include "generalGPIO.h"

//-------- PRIVATE METHODS --------//

rcl_interfaces::msg::SetParametersResult GeneralGPIOModule::octetParameterCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    // Handle the octet parameter change
    this->debugLog("Octet parameter changed to " + std::to_string(this->getOctet()));

    // Unsubscribe from the old response topic
    this->_response_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new response topic, at the new octet
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet" + std::to_string(this->getOctet()) + "_response", 10,
        std::bind(&GeneralGPIOModule::responseCallback, this, std::placeholders::_1));

    // Unsubscribe from the old sysadmin response topic
    this->_sysadmin_response_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new sysadmin response topic, at the new octet
    this->_sysadmin_response_subscription_ =
        this->create_subscription<roi_ros::msg::SerializedPacket>(
            "sys_admin_octet" + std::to_string(this->getOctet()) + "_response", 10,
            std::bind(&GeneralGPIOModule::sysadminResponseCallback, this, std::placeholders::_1));

    // Publish a health message mentioning the octet change
    auto healthMsg = roi_ros::msg::Health();
    healthMsg.operation_status = 1;
    healthMsg.message = "Octet parameter changed to " + std::to_string(this->getOctet());
    this->_health_publisher_->publish(healthMsg);

    // synchronize with the new module
    this->pushState();

    this->debugLog("Octet parameter change handled");

    return rcl_interfaces::msg::SetParametersResult();
}

rcl_interfaces::msg::SetParametersResult GeneralGPIOModule::aliasParameterCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    // Handle the alias parameter change
    this->debugLog("Alias parameter changed to " + this->getAlias());

    // Publish a health message mentioning the alias change
    auto healthMsg = roi_ros::msg::Health();
    healthMsg.operation_status = 1;
    healthMsg.message = "Alias parameter changed to " + this->getAlias();
    this->_health_publisher_->publish(healthMsg);

    this->debugLog("Alias parameter change handled");

    return rcl_interfaces::msg::SetParametersResult();
}

void GeneralGPIOModule::maintainState() {
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
            statusPacket.setAdminMetaData(sysAdminConstants::NOCHAINMETA);
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

void GeneralGPIOModule::responseCallback(const roi_ros::msg::SerializedPacket response) {
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
        case GeneralGPIOConstants::READ:
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

void GeneralGPIOModule::sysadminResponseCallback(const roi_ros::msg::SerializedPacket response) {
    // Handle the response from the sysadmin agent
    this->debugLog("Received response from sysadmin agent");

    // Parse the response packet
    ROIPackets::sysAdminPacket packet = ROIPackets::sysAdminPacket();
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
        case sysAdminConstants::STATUSREPORT: {
            if (data[0] == statusReportConstants::BLANKSTATE) {
                this->debugLog("Module reset detected, pushing state");
                this->pushState();
            }

            // update health message
            auto healthMsg = roi_ros::msg::Health();
            healthMsg.operation_status = data[0];
            healthMsg.message = this->_statusReportToHealthMessage(data[0]);

            // Publish the health message
            this->_health_publisher_->publish(healthMsg);
            break;
        }

        default:
            this->debugLog("Unknown sysadmin action code received: " +
                           std::to_string(packet.getActionCode()));
            break;
    }

    this->debugLog("Response handled");
}

void GeneralGPIOModule::publishPinStates() {
    // Publish the current pin states
    this->debugLog("Publishing Pin States");

    // Create the pin states message
    auto pinStatesMsg = roi_ros::msg::PinStates();
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        pinStatesMsg.states.push_back(this->subDeviceState[i]);
    }

    // Publish the pin states message
    this->_pinStatesPublisher->publish(pinStatesMsg);

    this->debugLog("Pin States Published");
}

void GeneralGPIOModule::publishPinValues() {
    // Publish the current pin values
    this->debugLog("Publishing Pin Values");

    // Create the pin values message
    auto pinValuesMsg = roi_ros::msg::PinValues();
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        pinValuesMsg.values.push_back(this->subDeviceValue[i]);
    }

    // Publish the pin values message
    this->_pinValuesPublisher->publish(pinValuesMsg);

    this->debugLog("Pin Values Published");
}

void GeneralGPIOModule::setPinOutputServiceHandler(
    const roi_ros::srv::SetPinOutput::Request::SharedPtr request,
    roi_ros::srv::SetPinOutput::Response::SharedPtr response) {
    // Handle the set pin output service request
    this->debugLog("Handling Set Pin Output Service Request");

    // Send the set pin output packet
    if (this->sendSetPinOutputPacket(request->pin, request->value)) {
        response->success = true;
    } else {
        response->success = false;
    }

    this->debugLog("Set Pin Output Service Request Handled");
}

void GeneralGPIOModule::setPinStateServiceHandler(
    const roi_ros::srv::SetPinState::Request::SharedPtr request,
    roi_ros::srv::SetPinState::Response::SharedPtr response) {
    // Handle the set pin state service request
    this->debugLog("Handling Set Pin State Service Request");

    // Send the set pin state packet
    if (this->sendSetPinStatePacket(request->pin, request->state)) {
        response->success = true;
    } else {
        response->success = false;
    }

    this->debugLog("Set Pin State Service Request Handled");
}

//-------- PUBLIC METHODS --------//

GeneralGPIOModule::GeneralGPIOModule() : BaseModule("generalGPIOModule") {
    // Initialize the GPIO module
    this->debugLog("Initializing General GPIO Module");

    // Initialize the GPIO module parameters and callbacks
    this->declare_parameter("module_alias", "blankGeneralGPIO");
    this->declare_parameter("module_octet", 5);
    this->_octetParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&GeneralGPIOModule::octetParameterCallback, this, std::placeholders::_1));
    this->_aliasParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&GeneralGPIOModule::aliasParameterCallback, this, std::placeholders::_1));

    // Initialize the GPIO module health publisher
    this->_health_publisher_ = this->create_publisher<roi_ros::msg::Health>("health", 10);

    // Initialize the GPIO module general packet queue client
    this->_queue_general_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedGeneralPacket>("queue_general_packet");

    // Initialize the GPIO module response subscriptions
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet5_response", 10,
        std::bind(&GeneralGPIOModule::responseCallback, this, std::placeholders::_1));
    this->_sysadmin_response_subscription_ =
        this->create_subscription<roi_ros::msg::SerializedPacket>(
            "sys_admin_octet5_response", 10,
            std::bind(&GeneralGPIOModule::sysadminResponseCallback, this, std::placeholders::_1));

    // Initialize the GPIO module specific publishers
    this->_pinStatesPublisher = this->create_publisher<roi_ros::msg::PinStates>("pin_states", 10);
    this->_pinValuesPublisher = this->create_publisher<roi_ros::msg::PinValues>("pin_values", 10);

    this->_setPinOutputService = this->create_service<roi_ros::srv::SetPinOutput>(
        "set_pin_output", std::bind(&GeneralGPIOModule::setPinOutputServiceHandler, this,
                                    std::placeholders::_1, std::placeholders::_2));
    this->_setPinStateService = this->create_service<roi_ros::srv::SetPinState>(
        "set_pin_state", std::bind(&GeneralGPIOModule::setPinStateServiceHandler, this,
                                   std::placeholders::_1, std::placeholders::_2));

    // Initialize the GPIO module maintain state thread
    this->_maintainStateThread = std::thread(&GeneralGPIOModule::maintainState, this);
}

GeneralGPIOModule::~GeneralGPIOModule() {
    // Destroy the GPIO module
    this->debugLog("Destroying General GPIO Module");
    this->_maintainStateThread
        .join();  // Wait for the maintain state thread to finish (It should stop itself)
}

bool GeneralGPIOModule::pushState() {
    // Push the current state of the GPIO module to the physical module

    // Loop through all the subdevices and push the state
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        this->sendSetPinStatePacket(i, this->subDeviceState[i]);
    }

    this->debugLog("GPIO Module State Pushed");

    return true;
}

bool GeneralGPIOModule::pullState() {
    // Pull the current state of the GPIO module from the physical module
    // NONE todo. Pull state is not implemented for the general GPIO module
    return false;
}

bool GeneralGPIOModule::sendSetPinStatePacket(uint8_t pin, uint8_t state) {
    // Send a packet changing the state of a pin
    this->debugLog("Sending Set Pin State Packet");

    // Create the set pin state packet
    ROIPackets::Packet setPinStatePacket = ROIPackets::Packet();
    setPinStatePacket.setActionCode(GeneralGPIOConstants::SET_PIN_MODE);
    setPinStatePacket.setSubDeviceID(pin);
    uint8_t data[1] = {state};
    setPinStatePacket.setData(data, 1);

    // Update the local store and publish the pin states
    subDeviceState[pin] = state;
    this->publishPinStates();

    // Send the set pin state packet
    if (this->sendGeneralPacket(setPinStatePacket)) {
        this->debugLog("Set Pin State Packet Sent");
        return true;
    } else {
        this->debugLog("Failed to send Set Pin State Packet");
        return false;
    }
}

bool GeneralGPIOModule::sendSetPinOutputPacket(uint8_t pin, uint8_t output) {
    // Send a packet changing the output of a pin
    this->debugLog("Sending Set Pin Output Packet");

    // Create the set pin output packet
    ROIPackets::Packet setPinOutputPacket = ROIPackets::Packet();
    setPinOutputPacket.setActionCode(GeneralGPIOConstants::SET_OUTPUT);
    setPinOutputPacket.setSubDeviceID(pin);
    uint8_t data[1] = {output};
    setPinOutputPacket.setData(data, 1);

    // Update the local store and publish the pin values
    subDeviceValue[pin] = output;
    this->publishPinValues();

    // Send the set pin output packet
    if (this->sendGeneralPacket(setPinOutputPacket)) {
        this->debugLog("Set Pin Output Packet Sent");
        return true;
    } else {
        this->debugLog("Failed to send Set Pin Output Packet");
        return false;
    }
}

bool GeneralGPIOModule::sendReadPinPacket(uint8_t pin) {
    // Send a packet reading the value of a pin
    this->debugLog("Sending Read Pin Packet");

    // Create the read pin packet
    ROIPackets::Packet readPinPacket = ROIPackets::Packet();
    readPinPacket.setActionCode(GeneralGPIOConstants::READ);
    readPinPacket.setSubDeviceID(pin);

    // Send the read pin packet
    if (this->sendGeneralPacket(readPinPacket)) {
        this->debugLog("Read Pin Packet Sent");
        return true;
    } else {
        this->debugLog("Failed to send Read Pin Packet");
        return false;
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeneralGPIOModule>());
    rclcpp::shutdown();
    return 0;
}