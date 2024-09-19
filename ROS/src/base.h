#ifndef BASEMODULE_H
#define BASEMODULE_H

#include <string>
#include <thread>

#include "../../lib/ModuleCodec.h"
#include "../../lib/Packet.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roi_ros/msg/health.hpp"
#include "roi_ros/msg/serialized_packet.hpp"
#include "roi_ros/srv/queue_serialized_general_packet.hpp"

/*
This is the base module abstract class for all virtual module modes (See virtualization layer). It
defines base interface functions that all virtual modules must implement.
*/

class BaseModule : public rclcpp::Node {
   protected:
    rclcpp::Parameter _moduleAliasParameter;  // The module alias parameter of the module
    rclcpp::Parameter _moduleOctetParameter;  // The module octet parameter of the module

    rclcpp::Publisher<roi_ros::msg::Health>::SharedPtr
        _health_publisher_;  // The health publisher of the module

    rclcpp::Client<roi_ros::srv::QueueSerializedGeneralPacket>::SharedPtr
        _queue_general_packet_client_;  // The general packet queue client of the module

    rclcpp::Subscription<roi_ros::msg::SerializedPacket>::SharedPtr
        _response_subscription_;  // The response subscription of the module

    /**
     * @brief A callback function for the module to handle octet parameter changes
     *
     * @param parameter
     * @return * rcl_interfaces::msg::SetParametersResult
     */
    virtual rcl_interfaces::msg::SetParametersResult octetParameterCallback(
        const rclcpp::Parameter &parameter) = 0;

    OnSetParametersCallbackHandle::SharedPtr _octetParameterCallbackHandle;  // The octet parameter
                                                                             // callback handle

    /**
     * @brief A callback function for the module to handle alias parameter changes
     *
     * @param parameter
     * @return * rcl_interfaces::msg::SetParametersResult
     */
    virtual rcl_interfaces::msg::SetParametersResult aliasParameterCallback(
        const rclcpp::Parameter &parameter) = 0;

    OnSetParametersCallbackHandle::SharedPtr _aliasParameterCallbackHandle;  // The alias parameter
                                                                             // callback handle

    /**
     * @brief A worker function for the module to maintain its state, in a separate thread
     *
     */
    virtual void maintainState() = 0;

    std::thread _maintainStateThread;  // Thread for the maintainState function

    /**
     * @brief Logs a message to the ros2 console
     *
     * @param message , std::string, the message to log
     */
    void debugLog(std::string message);

    /**
     * @brief Sends a general packet to the transport agent
     *
     * @param packet , ROIPackets::Packet, the packet to send
     */
    bool sendGeneralPacket(ROIPackets::Packet packet);

    /**
     * @brief Callback for the response from the transport agent when a response is received
     *
     * @param response , roi_ros::msg::SerializedPacket, the response packet
     */
    virtual void responseCallback(const roi_ros::msg::SerializedPacket response) = 0;

   public:
    /**
     * @brief Get the module host address octet
     *
     * @return uint8_t, the host address octet
     */
    uint8_t getOctet();

    /**
     * @brief Get the Alias string object
     *
     * @return std::string
     */
    std::string getAlias();

    virtual bool pushState() = 0;  // Pushes the current state of the module to the physical module
    virtual bool pullState() = 0;  // Pulls the current state of the module from the physical module
};

#endif  // BASEMODULE_H