#ifndef BASEMODULE_H
#define BASEMODULE_H

#include <string>
#include <thread>

#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/UnityTypes.hpp"
#include "../socketwrapper-2/include/socketwrapper/endpoint.hpp"
#include "../socketwrapper-2/include/socketwrapper/socketwrapper.hpp"
#include "../socketwrapper-2/include/socketwrapper/span.hpp"
#include "../socketwrapper-2/include/socketwrapper/udp.hpp"
#include "../socketwrapper-2/include/socketwrapper/utility.hpp"
#include "rclcpp/rclcpp.hpp"
/*

This is the base module template* for all virtual modules (See virtualization layer). It defines
base interface functions that all virtual modules must implement.

*not a cpp template, but a template for developers to use when creating new virtual modules.

*/

class BaseModule : public rclcpp::Node {
   protected:
    uint8_t _moduleOctet;  // The module octet of the module

    std::string _moduleAlias;  // The module alias of the module

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _health_publisher_;

    /**
     * @brief A worker function for the module to maintain its state, in a separate thread
     *
     */
    virtual void maintainState() = 0;  // ie refresh any data coming from the physical
                                       //  module; keep read values up to date.

    std::thread _maintainStateThread;  // Thread for the maintainState function

    /**
     * @brief Logs a message to the ros2 console
     *
     * @param message , std::string, the message to log
     */
    void debugLog(std::string message);

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

    friend class TransportAgent;  // TransportAgent needs access to the
                                  // ResponseCallback and MaintainState functions
};

class ROINode : public rclcpp::Node {
   public:
    ROINode() : Node("roi_integration") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

   private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);

        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

#endif  // BASEMODULE_H