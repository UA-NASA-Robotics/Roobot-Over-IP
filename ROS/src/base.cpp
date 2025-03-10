#include "base.h"

void BaseModule::debugLog(std::string message) { RCLCPP_INFO(this->get_logger(), message.c_str()); }

rcl_interfaces::msg::SetParametersResult BaseModule::octetParameterCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    // Handle the octet parameter change
    this->debugLog("Octet parameter changed to " + std::to_string(this->getOctet()));

    // Unsubscribe from the old response topic
    this->_response_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new response topic, at the new octet
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet" + std::to_string(this->getOctet()) + "_response", 10,
        std::bind(&BaseModule::responseCallback, this, std::placeholders::_1));

    // Unsubscribe from the old sysadmin response topic
    this->_sysadmin_response_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new sysadmin response topic, at the new octet
    this->_sysadmin_response_subscription_ =
        this->create_subscription<roi_ros::msg::SerializedPacket>(
            "sys_admin_octet" + std::to_string(this->getOctet()) + "_response", 10,
            std::bind(&BaseModule::sysadminResponseCallback, this, std::placeholders::_1));

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

bool BaseModule::sendGeneralPacket(ROIPackets::Packet packet) {
    auto request = std::make_shared<roi_ros::srv::QueueSerializedGeneralPacket::Request>();
    uint8_t serializedData[ROIConstants::ROI_MAX_PACKET_SIZE];
    packet.exportPacket(serializedData, ROIConstants::ROI_MAX_PACKET_SIZE);
    request->packet.data =
        std::vector<uint8_t>(serializedData, serializedData + ROIConstants::ROI_MAX_PACKET_SIZE);
    request->packet.length = ROIConstants::ROI_MAX_PACKET_SIZE;
    request->packet.client_octet = this->getOctet();
    auto result = this->_queue_general_packet_client_->async_send_request(request);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base =
        this->get_node_base_interface();
    if (rclcpp::spin_until_future_complete(
            node_base, result) ==  // wait for the result to return. Asyc function I.g.
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Packet queued to transportAgent");
        return result.get()->success;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to queue packet to transportAgent");
        return false;
    }
    return false;
}

bool BaseModule::sendSysadminPacket(ROIPackets::Packet packet) {
    auto request = std::make_shared<roi_ros::srv::QueueSerializedSysAdminPacket::Request>();
    uint8_t serializedData[ROIConstants::ROI_MAX_PACKET_SIZE];
    packet.exportPacket(serializedData, ROIConstants::ROI_MAX_PACKET_SIZE);
    request->packet.data =
        std::vector<uint8_t>(serializedData, serializedData + ROIConstants::ROI_MAX_PACKET_SIZE);
    request->packet.length = ROIConstants::ROI_MAX_PACKET_SIZE;
    request->packet.client_octet = this->getOctet();
    auto result = this->_queue_sysadmin_packet_client_->async_send_request(request);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base =
        this->get_node_base_interface();
    if (rclcpp::spin_until_future_complete(
            node_base, result) ==  // wait for the result to return. Asyc function I.g.
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Packet queued to transportAgent");
        return result.get()->success;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to queue packet to transportAgent");
        return false;
    }
    return false;
}

void BaseModule::connectionStateCallback(
    const roi_ros::msg::ConnectionState::SharedPtr connectionState) {
    bool updateHealth = false;
    if (this->_isConnected != connectionState->module_connected ||
        this->_lostPacketsSinceLastConnection != connectionState->lost_packets_since_connect ||
        this->_lostPacketsAccumulated != connectionState->lost_packets_accumulated) {
        updateHealth = true;
    }

    this->_isConnected = connectionState->module_connected;
    this->_lostPacketsSinceLastConnection = connectionState->lost_packets_since_connect;
    this->_lostPacketsAccumulated = connectionState->lost_packets_accumulated;

    if (updateHealth) {  // if the connection state has changed publish a health message
        this->publishHealthMessage();
    }
}

void BaseModule::sysadminResponseCallback(const roi_ros::msg::SerializedPacket response) {
    // Handle the response from the sysadmin agent
    this->debugLog("Received response from sysadmin agent");

    // Parse the response packet
    ROIPackets::sysAdminPacket packet = ROIPackets::sysAdminPacket();
    uint8_t serializedData[ROIConstants::ROI_MAX_PACKET_SIZE];
    this->unpackVectorToArray(response.data, serializedData, response.length);
    if (!packet.importPacket(serializedData, response.length) &&
        moduleNodeConstants::IGNORE_MALFORMED_PACKETS) {
        this->debugLog("Failed to import packet");
        return;
    }

    // Handle the response packet
    uint8_t data[ROIConstants::ROI_MAX_PACKET_PAYLOAD];
    packet.getData(data, ROIConstants::ROI_MAX_PACKET_PAYLOAD);
    switch (packet.getActionCode()) {
        case sysAdminConstants::STATUS_REPORT: {
            if (data[0] == statusReportConstants::BLANK_STATE) {
                this->debugLog("Module reset detected, pushing state");
                this->pushState();
                if (!this->_rosNodeInitialized) {
                    this->debugLog(
                        "ROS not initialized, marking as initialized as both are in blank state");
                    this->_rosNodeInitialized = true;
                }
            } else if (data[0] != statusReportConstants::BLANK_STATE &&
                       !this->_rosNodeInitialized) {
                // If the module is not in a blank state, and the ros node is not initialized, pull
                // state to recover into the ros node
                this->debugLog("Module not in blank state, and ROS not initalized, pulling state");
                this->pullState();
                this->_rosNodeInitialized = true;
            }

            _module_state = data[0];
            _module_operational = data[0] >= 1 && data[0] <= 3;
            _module_error = data[0] == 2 || data[0] == 4;
            _module_error_message = _statusReportToHealthMessage(data[0]);

            _timeAliveHours = data[1];
            _timeAliveMinutes = data[2];
            _timeAliveSeconds = data[3];

            _supplyVoltage = (float)(data[4] << 8 | data[5]) / 100;

            if (data[6] != _moduleType) {
                this->debugLog("Module type mismatch");
                _module_error = true;
                _module_error_message =
                    "Module type mismatch. Ensure the correct module is connected. Check IP octet";
            }

            for (int i = 0; i < 6; i++) {
                _mac[i] = data[i + 7];
            }

            this->publishHealthMessage();
            break;
        }

        case sysAdminConstants::BLACK_LIST:
            this->debugLog("Blacklist packet received");
            break;

        case sysAdminConstants::PING: {
            this->debugLog("Ping received, ponging back");
            ROIPackets::sysAdminPacket pongPacket = packet.swapReply();
            this->sendSysadminPacket(pongPacket);
            break;
        }

        case sysAdminConstants::PONG:
            this->debugLog("Pong received.");
            break;

        default:
            this->debugLog("Unknown sysadmin action code received: " +
                           std::to_string(packet.getActionCode()));
            break;
    }

    this->debugLog("Response handled");
}

void BaseModule::publishHealthMessage() {
    auto message = roi_ros::msg::Health();
    message.module_connection = _isConnected;
    message.module_operational = _module_operational;
    message.module_state = _module_state;
    message.module_error = _module_error;
    message.module_error_message = _module_error_message;

    message.time_alive_hours = _timeAliveHours;
    message.time_alive_minutes = _timeAliveMinutes;
    message.time_alive_seconds = _timeAliveSeconds;

    message.supply_voltage = _supplyVoltage;

    for (int i = 0; i < 6; i++) {
        message.mac.push_back(_mac[i]);
    }
    this->_health_publisher_->publish(message);
}

void BaseModule::unpackVectorToArray(std::vector<uint8_t> vector, uint8_t *array,
                                     uint16_t arraySize) {
    for (int i = 0; i < arraySize; i++) {
        array[i] = vector[i];
    }
}

std::string BaseModule::_statusReportToHealthMessage(uint8_t statusReport) {
    switch (statusReport) {
        case 0:
            return "Null Code, Miscommunication, check code.";
        case 1:
            return "OK";
        case 2:
            return "Operational, internal error, check hardware.";
        case 3:
            return "Operational, no chain formed.";
        case 4:
            return "Not operable, hard error.";
        case 5:
            return "Initializing, not ready for operation.";
        case 6:
            return "Blank state, Device is ready to operate, but requires configuration before "
                   "use.";
        default:
            return "Unknown";
    }
}

uint8_t BaseModule::getOctet() { return this->get_parameter("module_octet").as_int(); }

BaseModule::BaseModule(std::string nodeName, const uint8_t moduleType)
    : Node(nodeName), _moduleType(moduleType) {
    // Initialize the network parameters and callbacks
    this->declare_parameter("module_octet", 5);
    this->_octetParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&BaseModule::octetParameterCallback, this, std::placeholders::_1));

    // Initialize the module health publisher
    this->_health_publisher_ = this->create_publisher<roi_ros::msg::Health>("health", 10);

    // Initialize the module general packet queue client
    this->_queue_general_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedGeneralPacket>("queue_general_packet");

    this->_queue_sysadmin_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedSysAdminPacket>("queue_sys_admin_packet");

    // Initialize the base module response subscriptions (gets data from the transport agent)
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet5_response", 10,
        std::bind(&BaseModule::responseCallback, this, std::placeholders::_1));
    this->_sysadmin_response_subscription_ =
        this->create_subscription<roi_ros::msg::SerializedPacket>(
            "sys_admin_octet5_response", 10,
            std::bind(&BaseModule::sysadminResponseCallback, this, std::placeholders::_1));

    this->_connection_state_subscription_ =
        this->create_subscription<roi_ros::msg::ConnectionState>(
            "octet5_connection_state", 10,
            std::bind(&BaseModule::connectionStateCallback, this, std::placeholders::_1));
};

BaseModule::~BaseModule() {
    this->_maintainStateThread.join();
    this->debugLog("Destroying Base Module");
}