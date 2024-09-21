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
        // Check if the module has been reset

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