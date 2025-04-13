#ifndef GENERALGPIO_H
#define GENERALGPIO_H

#include "base.h"
#include "roi_ros/msg/pin_states.hpp"
#include "roi_ros/msg/pin_values.hpp"
#include "roi_ros/srv/set_pin_output.hpp"
#include "roi_ros/srv/set_pin_state.hpp"

class GeneralGPIOModule : public BaseModule {
   protected:
    // Msg publishers
    rclcpp::Publisher<roi_ros::msg::PinStates>::SharedPtr _pin_states_publisher_;
    rclcpp::Publisher<roi_ros::msg::PinValues>::SharedPtr _pin_values_publisher_;

    // Service servers
    rclcpp::Service<roi_ros::srv::SetPinState>::SharedPtr _set_pin_state_service_;
    rclcpp::Service<roi_ros::srv::SetPinOutput>::SharedPtr _set_pin_output_service_;

    // State Duplication (used for reference, and pushpull state)
    uint8_t _pin_states[GeneralGPIOConstants::COUNT];
    uint8_t _pin_values[GeneralGPIOConstants::COUNT];

    /**
     * @brief A worker function for the module to maintain its state, in a separate thread
     *  For the general GPIO module, this function will be used to refresh input values.
     *  It will also push the current state to the physical module if the module is reset.
     */

    void maintainState() override;

    /**
     * @brief Callback for the response from the transport agent when a response is received
     *
     * @param response , roi_ros::msg::SerializedPacket, the response packet
     */
    void responseCallback(const roi_ros::msg::SerializedPacket response) override;

    /**
     * @brief Publishes the pin states message
     *
     */
    void publishPinStatesMessage();

    /**
     * @brief Publishes the pin values message
     *
     */
    void publishPinValuesMessage();

    /**
     * @brief Callback for the set pin state service
     *
     * @param request , roi_ros::srv::SetPinState::Request, the request message
     * @param response , roi_ros::srv::SetPinState::Response, the response message
     */
    void setPinStateServiceHandler(const roi_ros::srv::SetPinState::Request::SharedPtr request,
                                   roi_ros::srv::SetPinState::Response::SharedPtr response);

    /**
     * @brief  Callback for the set pin output service
     *
     * @param request , roi_ros::srv::SetPinOutput::Request, the request message
     * @param response , roi_ros::srv::SetPinOutput::Response, the response message
     */
    void setPinOutputServiceHandler(const roi_ros::srv::SetPinOutput::Request::SharedPtr request,
                                    roi_ros::srv::SetPinOutput::Response::SharedPtr response);

    /**
     * @brief Send Set Pin State Packet
     *
     * @param pin , uint8_t, the pin to set
     * @param state , uint8_t, the state to set
     */
    void sendSetPinStatePacket(uint8_t pin, uint8_t state);

    /**
     * @brief Send Set Pin Output Packet
     *
     * @param pin , uint8_t, the pin to set
     * @param value, bool, the value to set
     */
    void sendSetPinOutputPacket(uint8_t pin, bool state);

    /**
     * @brief Send Read Pin Packet
     *
     * @param pin , uint8_t, the pin to read
     */
    void sendReadPinPacket(uint8_t pin);

    /**
     * @brief Send get Pin States Packet
     *
     *
     */
    void sendGetPinStatesPacket();

    /**
     * @brief Send get Pin Output Packet
     *
     *
     */
    void sendGetPinOutputPacket();

   public:
    GeneralGPIOModule();
    ~GeneralGPIOModule();

    /**
     * @brief Pushes the current state of the GeneralGPIO module to the physical module
     *
     * @return true, if the state was successfully pushed
     * @return false, if the state was not successfully pushed
     */
    bool pushState() override;
    /**
     * @brief Pulls the current state of the GeneralGPIO module from the physical module
     *
     * @return true, if the state was successfully pulled
     * @return false, if the state was not successfully pulled
     */
    bool pullState() override;
};

#endif