#include "base.h"

void BaseModule::debugLog(std::string message) { RCLCPP_INFO(this->get_logger(), message.c_str()); }

bool BaseModule::sendGeneralPacket(ROIPackets::Packet packet) {
    auto request = std::make_shared<roi_ros::srv::QueueSerializedGeneralPacket::Request>();
    uint8_t serializedData[ROIConstants::ROIMAXPACKETSIZE];
    packet.exportPacket(serializedData, ROIConstants::ROIMAXPACKETSIZE);
    request->packet.data =
        std::vector<uint8_t>(serializedData, serializedData + ROIConstants::ROIMAXPACKETSIZE);
    request->packet.length = ROIConstants::ROIMAXPACKETSIZE;
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

uint8_t BaseModule::getOctet() { return this->get_parameter("module_octet").as_int(); }

std::string BaseModule::getAlias() { return this->get_parameter("module_alias").as_string(); }
