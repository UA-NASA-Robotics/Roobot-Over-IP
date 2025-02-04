#ifndef BASEMODULE_H
#define BASEMODULE_H

#include <string>
#include <thread>

#include "../../lib/ModuleCodec.h"
#include "../../lib/Packet.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "roi_ros/msg/connection_state.hpp"
#include "roi_ros/msg/health.hpp"
#include "roi_ros/msg/serialized_packet.hpp"
#include "roi_ros/srv/queue_serialized_general_packet.hpp"
#include "roi_ros/srv/queue_serialized_sys_admin_packet.hpp"

/*
This is the base module abstract class for all virtual module modes (See virtualization layer). It
defines base interface functions that all virtual modules must implement.
*/

namespace moduleNodeConstants {
constexpr bool ignoreMalformedPackets = false;  // Ignore malformed packets (failed checksums)
}  // namespace moduleNodeConstants

class BaseModule : public rclcpp::Node {
   protected:
    rclcpp::Publisher<roi_ros::msg::Health>::SharedPtr
        _health_publisher_;  // The health publisher of the module

    rclcpp::Client<roi_ros::srv::QueueSerializedGeneralPacket>::SharedPtr
        _queue_general_packet_client_;  // The general packet queue client of the module

    rclcpp::Client<roi_ros::srv::QueueSerializedSysAdminPacket>::SharedPtr
        _queue_sysadmin_packet_client_;  // The sysadmin packet queue client of the module

    rclcpp::Subscription<roi_ros::msg::SerializedPacket>::SharedPtr
        _response_subscription_;  // The response subscription of the module

    rclcpp::Subscription<roi_ros::msg::SerializedPacket>::SharedPtr
        _sysadmin_response_subscription_;  // The sysadmin response subscription of the module

    rclcpp::Subscription<roi_ros::msg::ConnectionState>::SharedPtr
        _connection_state_subscription_;  // The connection state subscription of the module

    /**
     * @brief A callback function for the module to handle octet parameter changes
     *
     * @param parameter
     * @return * rcl_interfaces::msg::SetParametersResult
     */
    virtual rcl_interfaces::msg::SetParametersResult octetParameterCallback(
        const std::vector<rclcpp::Parameter> &parameters) = 0;

    OnSetParametersCallbackHandle::SharedPtr _octetParameterCallbackHandle;  // The octet parameter
                                                                             // callback handle

    /**
     * @brief A worker function for the module to maintain its state, in a separate thread
     *
     */
    virtual void maintainState() = 0;

    std::thread _maintainStateThread;

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
     * @brief Sends a sysadmin packet to the sysadmin agent
     *
     * @param packet , ROIPackets::Packet, the packet to send
     */
    bool sendSysadminPacket(ROIPackets::Packet packet);

    /**
     * @brief Callback for the response from the transport agent when a response is received
     *
     * @param response , roi_ros::msg::SerializedPacket, the response packet
     */
    virtual void responseCallback(const roi_ros::msg::SerializedPacket response) = 0;

    /**
     * @brief Callback for the response from the sysadmin agent when a response is received
     *
     * @param response
     */
    virtual void sysadminResponseCallback(const roi_ros::msg::SerializedPacket response) = 0;

    /**
     * @brief Implement a fuction that publishes a health update including all relevant information
     * @breif This function is utilized by the connectionState Subscription when necessary
     *
     */
    virtual void publishHealthMessage() = 0;

    /**
     * @brief Callback for the connection state message
     *
     * @param connectionState , roi_ros::msg::ConnectionState, the connection state message
     */
    void connectionStateCallback(const roi_ros::msg::ConnectionState::SharedPtr connectionState);

    /**
     * @brief Unpacks a vector into an array
     *
     * @param vector , std::vector<uint8_t>, the vector to unpack
     * @param array , uint8_t*, the array to unpack to
     * @param arraySize , uint16_t, the size of the array or how many to unpack from the vector
     *
     * @return void
     */
    void unpackVectorToArray(std::vector<uint8_t> vector, uint8_t *array, uint16_t arraySize);

    /**
     * @brief Converts a status report to a health message string
     *
     * @param statusReport , uint8_t, the status report to convert
     * @return std::string, the health message
     */
    std::string _statusReportToHealthMessage(uint8_t statusReport);

    // fields

    bool _isConnected;  // The connection state of the module
    uint32_t
        _lostPacketsSinceLastConnection;  // The number of lost packets since the last connection
    uint32_t _lostPacketsAccumulated;     // The total number of lost packets

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

    BaseModule(std::string moduleName);  // Constructor
    ~BaseModule();                       // Destructor
};

#endif  // BASEMODULE_H