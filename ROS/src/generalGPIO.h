#ifndef GENERALGPIO_H
#define GENERALGPIO_H

#include "base.h"
#include "roi_ros/msg/PinStates.msg"
#include "roi_ros/msg/PinValues.hpp"
#include "roi_ros/srv/set_pin_output.hpp"
#include "roi_ros/srv/set_pin_state.hpp"

class generalGPIOModule : public BaseModule {
   protected:
    uint8_t subDeviceState[GeneralGPIOConstants::COUNT] = {GeneralGPIOConstants::INPUT_MODE};
    uint16_t subDeviceValue[GeneralGPIOConstants::COUNT] = {0};

    // Value Topics
    rclcpp::Publisher<roi_ros::msg::PinStates>::SharedPtr _pinStatesPublisher;
    rclcpp::Publisher<roi_ros::msg::PinValues>::SharedPtr _pinValuesPublisher;

    // Service Msgs
    rclcpp::Service<roi_ros::srv::SetPinOutput>::SharedPtr _setPinOutputService;
    rclcpp::Service<roi_ros::srv::SetPinState>::SharedPtr _setPinStateService;

    /**
     * @brief A callback function for the module to handle octet parameter changes
     *
     * @param parameter
     * @return * rcl_interfaces::msg::SetParametersResult
     */
    rcl_interfaces::msg::SetParametersResult octetParameterCallback(
        const rclcpp::Parameter &parameter);

    /**
     * @brief A callback function for the module to handle alias parameter changes
     *
     * @param parameter
     * @return * rcl_interfaces::msg::SetParametersResult
     */
    rcl_interfaces::msg::SetParametersResult aliasParameterCallback(
        const rclcpp::Parameter &parameter);

    /**
     * @brief A worker function for the module to maintain its state, in a separate thread
     *  For the general GPIO module, this function will be used to refresh input values.
     *  It will also push the current state to the physical module if the module is reset.
     */

    void maintainState();

    /**
     * @brief Callback for the response from the transport agent when a response is received
     *
     * @param response , roi_ros::msg::SerializedPacket, the response packet
     */
    void responseCallback(const roi_ros::msg::SerializedPacket response);

    /**
     * @brief Updates ROS2 topics with the current state of the GPIO module
     *
     */
    void publishPinStates();

    /**
     * @brief Updates ROS2 topics with the current values of the GPIO module
     *
     */
    void publishPinValues();

   public:
    generalGPIOModule();
    ~generalGPIOModule();

    /**
     * @brief Pushes the current state of the GPIO module to the physical module
     *
     * @return true, if the state was successfully pushed
     * @return false, if the state was not successfully pushed
     */
    bool pushState();
    /**
     * @brief WIP, Pulls the current state of the GPIO module from the physical module
     *
     * @return true, if the state was successfully pulled
     * @return false, if the state was not successfully pulled
     */
    bool pullState();

    /**
     * @brief Sends a packet changing the state of a pin
     *
     * @param pin , uint8_t, the pin to change the state of
     * @param state , uint8_t, the state to change the pin to
     * @return true, if the packet was successfully sent
     * @return false, if the packet was not successfully sent
     */
    bool sendSetPinStatePacket(uint8_t pin, uint8_t state);

    /**
     * @brief Sends a packet changing the output of a pin
     *
     * @param pin , uint8_t, the pin to change the output of
     * @param output , uint8_t, the output to change the pin to
     * @return true
     * @return false
     */
    bool sendSetPinOutputPacket(uint8_t pin, uint8_t output);

    /**
     * @brief Sends a packet to read the value of a pin
     *
     * @param pin , uint8_t, the pin to read the value of
     * @return true, if the packet was successfully sent
     * @return false , if the packet was not successfully sent
     */
    bool sendReadPinPacket(uint8_t pin);
};

#endif