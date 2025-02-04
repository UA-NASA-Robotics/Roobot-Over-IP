#include "ODrive.h"

//-------- PRIVATE METHODS --------//

rcl_interfaces::msg::SetParametersResult ODriveModule::octetParameterCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    // Handle the octet parameter change
    this->debugLog("Octet parameter changed to " + std::to_string(this->getOctet()));

    // Unsubscribe from the old response topic
    this->_response_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new response topic, at the new octet
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet" + std::to_string(this->getOctet()) + "_response", 10,
        std::bind(&ODriveModule::responseCallback, this, std::placeholders::_1));

    // Unsubscribe from the old sysadmin response topic
    this->_sysadmin_response_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new sysadmin response topic, at the new octet
    this->_sysadmin_response_subscription_ =
        this->create_subscription<roi_ros::msg::SerializedPacket>(
            "sys_admin_octet" + std::to_string(this->getOctet()) + "_response", 10,
            std::bind(&ODriveModule::sysadminResponseCallback, this, std::placeholders::_1));

    // Unsubscribe from the old connection state topic
    this->_connection_state_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new connection state topic, at the new octet
    this->_connection_state_subscription_ =
        this->create_subscription<roi_ros::msg::ConnectionState>(
            "octet" + std::to_string(this->getOctet()) + "_connection_state", 10,
            std::bind(&BaseModule::connectionStateCallback, this, std::placeholders::_1));

    // synchronize with the new module
    this->pushState();

    this->debugLog("Octet parameter change handled");

    return rcl_interfaces::msg::SetParametersResult();
}

void ODriveModule::maintainState() {
    // Maintain the state of the ODrive module

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
        checkResetCounter--;  // Increment the check reset counter

        // Loop through all of the readable values and request their values
        ROIPackets::Packet readPacket = ROIPackets::Packet();
        readPacket.setActionCode(ODriveConstants::GETALL);
        this->sendGeneralPacket(readPacket);

        // Sleep for n seconds
        std::this_thread::sleep_for(
            std::chrono::milliseconds(WatchdogConstants::MAINTAIN_SLEEP_TIME));
    }
}

void ODriveModule::responseCallback(const roi_ros::msg::SerializedPacket response) {
    // Handle the response from the transport agent
    this->debugLog("Received response from transport agent");

    // Parse the response packet
    ROIPackets::Packet packet = ROIPackets::Packet();
    uint8_t serializedData[ROIConstants::ROIMAXPACKETSIZE];
    this->unpackVectorToArray(response.data, serializedData, response.length);
    if (!packet.importPacket(serializedData, response.length) &&
        !moduleNodeConstants::ignoreMalformedPackets) {
        this->debugLog("Failed to import packet");
        return;
    }

    // Handle the response packet
    uint8_t data[ROIConstants::ROIMAXPACKETPAYLOAD];
    packet.getData(data, ROIConstants::ROIMAXPACKETPAYLOAD);
    if (packet.getActionCode() & ODriveConstants::MaskConstants::GETMASK) {
        // Handle the response to a get request
        switch (packet.getActionCode() & !ODriveConstants::MaskConstants::GETMASK) {
            case ODriveConstants::MaskConstants::ControlMode:
                _controlMode = data[0];
                break;

            case ODriveConstants::MaskConstants::InputMode:
                _inputMode = data[0];
                break;

            case ODriveConstants::MaskConstants::Torque:
                _inputTorque = floatCast::toFloat(data, 0, 3);
                break;

            case ODriveConstants::MaskConstants::PositionSetPoint:
                _inputPosition = floatCast::toFloat(data, 0, 3);
                break;

            case ODriveConstants::MaskConstants::VelocitySetPoint:
                _inputVelocity = floatCast::toFloat(data, 0, 3);
                break;

            case ODriveConstants::MaskConstants::Error:
                _errorCode = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];

                this->publishHealthMessage();
                break;

            case ODriveConstants::MaskConstants::Velocity:
                _velocity = floatCast::toFloat(data, 0, 3);

                this->publishStateMessage();
                break;

            case ODriveConstants::MaskConstants::Position:
                _position = floatCast::toFloat(data, 0, 3);

                this->publishStateMessage();
                break;

            case ODriveConstants::MaskConstants::BusVoltage:
                _busVoltage = floatCast::toFloat(data, 0, 3);

                this->publishPowerMessage();
                break;

            case ODriveConstants::MaskConstants::Current:
                _current = floatCast::toFloat(data, 0, 3);

                this->publishPowerMessage();
                break;

            case ODriveConstants::MaskConstants::MotorTemperature:
                _motorTemperature = floatCast::toFloat(data, 0, 3);

                this->publishTemperatureMessage();
                break;

            case ODriveConstants::MaskConstants::FETTemperature:
                _fetTemperature = floatCast::toFloat(data, 0, 3);

                this->publishTemperatureMessage();
                break;

            case ODriveConstants::MaskConstants::KinematicFeedback:
                _position = floatCast::toFloat(data, 0, 3);
                _velocity = floatCast::toFloat(data, 4, 7);

                this->publishStateMessage();
                break;

            case ODriveConstants::MaskConstants::all:
                _position = floatCast::toFloat(data, 0, 3);
                _velocity = floatCast::toFloat(data, 4, 7);
                _busVoltage = floatCast::toFloat(data, 8, 11);
                _current = floatCast::toFloat(data, 12, 15);
                _fetTemperature = floatCast::toFloat(data, 16, 19);
                _motorTemperature = floatCast::toFloat(data, 20, 23);

                _errorCode = data[24] << 24 | data[25] << 16 | data[26] << 8 | data[27];

                this->publishHealthMessage();
                this->publishPowerMessage();
                this->publishTemperatureMessage();
                this->publishStateMessage();

                break;

            default:
                this->debugLog("Unknown get action code received: " +
                               std::to_string(packet.getActionCode()));
                break;
        }
    } else {
        // Handle the response to a set request
        switch (packet.getActionCode() & !ODriveConstants::MaskConstants::GETMASK) {
            case ODriveConstants::MaskConstants::ControlMode:
                this->debugLog("Control mode set");
                if (!data[0]) {
                    this->debugLog("Controlmode set failure.");
                    _module_error_message = "Controlmode set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::InputMode:
                this->debugLog("Input mode set");
                if (!data[0]) {
                    this->debugLog("Input mode set failure.");
                    _module_error_message = "Input mode set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::Torque:
                this->debugLog("Torque set");
                if (!data[0]) {
                    this->debugLog("Torque set failure.");
                    _module_error_message = "Torque set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::PositionSetPoint:
                this->debugLog("Position set");
                if (!data[0]) {
                    this->debugLog("Position set failure.");
                    _module_error_message = "Position set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::VelocitySetPoint:
                this->debugLog("Velocity set");
                if (!data[0]) {
                    this->debugLog("Velocity set failure.");
                    _module_error_message = "Velocity set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::PositionRelative:
                this->debugLog("Relative position set");
                if (!data[0]) {
                    this->debugLog("Relative position set failure.");
                    _module_error_message = "Relative position set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::Error:
                this->debugLog("Error cleared");
                if (!data[0]) {
                    this->debugLog("Error clear failure.");
                    _module_error_message = "Error clear failure.";
                    this->publishHealthMessage();
                }
                break;

            default:
                this->debugLog("Unknown action code received: " +
                               std::to_string(packet.getActionCode()));
                break;
        }
    }

    this->debugLog("Response handled");
}

//-------- PUBLIC METHODS --------//

ODriveModule::ODriveModule() : BaseModule("ODriveModule") {
    // Initialize the GPIO module
    this->debugLog("Initializing General GPIO Module");

    // Initialize the GPIO module parameters and callbacks
    this->declare_parameter("module_alias", "blankGeneralGPIO");
    this->declare_parameter("module_octet", 5);
    this->_octetParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&ODriveModule::octetParameterCallback, this, std::placeholders::_1));
    this->_aliasParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&ODriveModule::aliasParameterCallback, this, std::placeholders::_1));

    // Initialize the GPIO module health publisher
    this->_health_publisher_ = this->create_publisher<roi_ros::msg::Health>("health", 10);

    // Initialize the GPIO module general packet queue client
    this->_queue_general_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedGeneralPacket>("queue_general_packet");

    this->_queue_sysadmin_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedSysAdminPacket>("queue_sys_admin_packet");

    // Initialize the GPIO module response subscriptions
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet5_response", 10,
        std::bind(&ODriveModule::responseCallback, this, std::placeholders::_1));
    this->_sysadmin_response_subscription_ =
        this->create_subscription<roi_ros::msg::SerializedPacket>(
            "sys_admin_octet5_response", 10,
            std::bind(&ODriveModule::sysadminResponseCallback, this, std::placeholders::_1));

    // Initialize the GPIO module maintain state thread
    this->_maintainStateThread = std::thread(&ODriveModule::maintainState, this);
}

ODriveModule::~ODriveModule() {
    // Destroy the GPIO module
    this->debugLog("Destroying General GPIO Module");
    this->_maintainStateThread
        .join();  // Wait for the maintain state thread to finish (It should stop itself)
}

bool ODriveModule::pushState() {
    // Push the current state of the GPIO module to the physical module

    // Loop through all the subdevices and push the state
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        this->sendSetPinStatePacket(i, this->subDeviceState[i]);
    }

    this->debugLog("GPIO Module State Pushed");

    return true;
}

bool ODriveModule::pullState() {
    // Pull the current state of the GPIO module from the physical module
    // NONE todo. Pull state is not implemented for the general GPIO module
    return false;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODriveModule>());
    rclcpp::shutdown();
    return 0;
}