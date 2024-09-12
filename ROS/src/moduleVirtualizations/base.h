#ifndef BASEMODULE_H
#define BASEMODULE_H

#include <string>
#include <thread>

#include ""
#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/UnityTypes.hpp"
#include "rclcpp/rclcpp.hpp"
/*
This is the base module abstract class for all virtual module modes (See virtualization layer). It
defines base interface functions that all virtual modules must implement.
*/

class BaseModule : public rclcpp::Node {
   protected:
    rclcpp::Parameter _moduleAliasParameter;  // The module alias parameter of the module
    rclcpp::Parameter _moduleOctetParameter;  // The module octet parameter of the module

    rclcpp::Publisher<ms*** * todo>::SharedPtr _health_publisher_;

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
};

class ROINode : public rclcpp::Node {
   public:
    ROINode() : Node("base_module") {
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