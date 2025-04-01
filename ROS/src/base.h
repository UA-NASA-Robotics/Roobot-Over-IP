#ifndef BASEMODULE_H
#define BASEMODULE_H

#include <string>
#include <thread>

#include "../../lib/ModuleCodec.h"
#include "../../lib/Packet.h"
#include "../../lib/floatCast.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "roi_interfaces/msg/connection_state.hpp"
#include "roi_interfaces/msg/health.hpp"
#include "roi_interfaces/msg/serialized_packet.hpp"
#include "roi_interfaces/srv/queue_serialized_general_packet.hpp"
#include "roi_interfaces/srv/queue_serialized_sys_admin_packet.hpp"

/*
This is the base module abstract class for all virtual module modes (See virtualization layer). It
defines base interface functions that all virtual modules must implement.
*/

namespace moduleNodeConstants {
constexpr bool IGNORE_MALFORMED_PACKETS = false;  // Ignore malformed packets (failed checksums)
}  // namespace moduleNodeConstants

struct healthData {
    bool _isConnected;  // The connection state of the module
    uint32_t
        _lostPacketsSinceLastConnection;  // The number of lost packets since the last connection
    uint32_t _lostPacketsAccumulated;     // The total number of lost packets

    bool _module_operational = false;
    uint16_t _module_state;
    bool _module_error = false;
    std::string _module_error_message;

    uint8_t _timeAliveHours;    // The number of hours the module has been alive
    uint8_t _timeAliveMinutes;  // The number of minutes the module has been alive
    uint8_t _timeAliveSeconds;  // The number of seconds the module has been alive

    float _supplyVoltage;  // The voltage of the module

    bool _rosNodeInitialized;  // used to determine if the ros node has been initialized, we can
                               // pull the state from the module if it has been initialized and the
                               // ros node is not.

    uint8_t _mac[6];  // The mac address of the module
};

class BaseModule : public rclcpp::Node {
   protected:
    rclcpp::Publisher<roi_interfaces::msg::Health>::SharedPtr
        _health_publisher_;  // The health publisher of the module

    rclcpp::Client<roi_interfaces::srv::QueueSerializedGeneralPacket>::SharedPtr
        _queue_general_packet_client_;  // The general packet queue client of the module

    rclcpp::Client<roi_interfaces::srv::QueueSerializedSysAdminPacket>::SharedPtr
        _queue_sysadmin_packet_client_;  // The sysadmin packet queue client of the module

    rclcpp::Subscription<roi_interfaces::msg::SerializedPacket>::SharedPtr
        _response_subscription_;  // The response subscription of the module

    rclcpp::Subscription<roi_interfaces::msg::SerializedPacket>::SharedPtr
        _sysadmin_response_subscription_;  // The sysadmin response subscription of the module

    rclcpp::Subscription<roi_interfaces::msg::ConnectionState>::SharedPtr
        _connection_state_subscription_;  // The connection state subscription of the module

    // Stored State Variables. Needed for health messages and other state related functions
    healthData _healthData;  // The health data of the module

    const uint8_t _moduleType;  // The type of the module (set at construction, used to check
    // coherency of module to ros connection)

    /**
     * @brief A callback function for the module to handle octet parameter changes
     *
     * @param parameter
     * @return * rcl_interfaces::msg::SetParametersResult
     */
    virtual rcl_interfaces::msg::SetParametersResult octetParameterCallback(
        const std::vector<rclcpp::Parameter> &parameters);

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
     * @param response , roi_interfaces::msg::SerializedPacket, the response packet
     */
    virtual void responseCallback(const roi_interfaces::msg::SerializedPacket response) = 0;

    /**
     * @brief Callback for the response from the sysadmin agent when a response is received
     *
     * @param response
     */
    void sysadminResponseCallback(const roi_interfaces::msg::SerializedPacket response);

    /**
     * @brief Callback for the connection state message
     *
     * @param connectionState , roi_interfaces::msg::ConnectionState, the connection state message
     */
    void connectionStateCallback(const roi_interfaces::msg::ConnectionState::SharedPtr connectionState);

    /**
     * @brief Implement a fuction that publishes a health update including all relevant information
     *
     *
     */
    void publishHealthMessage();

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

   public:
    /**
     * @brief Get the module host address octet
     *
     * @return uint8_t, the host address octet
     */
    uint8_t getOctet();

    virtual bool pushState() = 0;  // Pushes the current state of the module to the physical module
    virtual bool pullState() = 0;  // Pulls the current state of the module from the physical module

    BaseModule(std::string moduleName, const uint8_t moduleType);  // Constructor
    ~BaseModule();                                                 // Destructor
};

#endif  // BASEMODULE_H