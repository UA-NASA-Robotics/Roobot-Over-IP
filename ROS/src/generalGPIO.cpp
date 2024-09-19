#include "generalGPIO.h"

//-------- PRIVATE METHODS --------//

//-------- PUBLIC METHODS --------//

GeneralGPIO::GeneralGPIO() : BaseModule() {
    // Initialize the GPIO module
    this->debugLog("Initializing General GPIO Module");
    this->_moduleAliasParameter = this->declare_parameter("module_alias", "blankGeneralGPIO");
    this->_moduleOctetParameter = this->declare_parameter("module_octet", 5);
    this->_health_publisher_ = this->create_publisher<roi_ros::msg::Health>("health", 10);
    this->_queue_general_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedGeneralPacket>("queue_general_packet");
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "", 10, std::bind(&GeneralGPIO::responseCallback, this, std::placeholders::_1));
    this->_maintainStateThread = std::thread(&GeneralGPIO::maintainState, this);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeneralGPIO>());
    rclcpp::shutdown();
    return 0;
}