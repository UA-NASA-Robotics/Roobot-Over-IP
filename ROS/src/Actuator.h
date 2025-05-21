#ifndef GENERALGPIO_H
#define GENERALGPIO_H

#include "base.h"
#include "roi_ros/action/actuator_goto_absolute_position.hpp"
#include "roi_ros/action/actuator_goto_relative_position.hpp"
#include "roi_ros/msg/actuator_state.hpp"
#include "roi_ros/srv/actuator_goto_absolute_position.hpp"
#include "roi_ros/srv/actuator_goto_relative_position.hpp"
#include "roi_ros/srv/actuator_set_velocity.hpp"

class ActuatorModule : public BaseModule {
   protected:
    // Msg publishers
    rclcpp::Publisher<roi_ros::msg::ActuatorState>::SharedPtr _state_publisher_;

    // Service servers
    rclcpp::Service<roi_ros::srv::ActuatorGotoAbsolutePosition>::SharedPtr _goto_position_service_;
    rclcpp::Service<roi_ros::srv::ActuatorGotoRelativePosition>::SharedPtr
        _goto_relative_position_service_;
    rclcpp::Service<roi_ros::srv::ActuatorSetVelocity>::SharedPtr _set_velocity_service_;

    // State Duplication (used for reference, and pushpull state)
    uint8_t _controlMode;
    uint16_t _inputPosition;
    float _inputVelocity;

    uint16_t _position;
    float _velocity;

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
     * @brief Publishes the state message, position and velocity.
     *
     */
    void publishStateMessage();

    /**
     * @brief Callback for the goto position service
     *
     * @param request
     * @param response
     */
    void gotoAbsolutePositionServiceHandler(
        const roi_ros::srv::ActuatorGotoAbsolutePosition::Request::SharedPtr request,
        roi_ros::srv::ActuatorGotoAbsolutePosition::Response::SharedPtr response);

    /**
     * @brief Callback for the goto relative position service
     *
     * @param request
     * @param response
     */
    void gotoRelativePositionServiceHandler(
        const roi_ros::srv::ActuatorGotoRelativePosition::Request::SharedPtr request,
        roi_ros::srv::ActuatorGotoRelativePosition::Response::SharedPtr response);

    /**
     * @brief Callback for the set velocity service
     *
     * @param request
     * @param response
     */
    void setVelocityServiceHandler(
        const roi_ros::srv::ActuatorSetVelocity::Request::SharedPtr request,
        roi_ros::srv::ActuatorSetVelocity::Response::SharedPtr response);

    void sendGotoAbsolutePositionPacket(uint16_t position, uint16_t subDeviceID);

    void sendGotoRelativePositionPacket(uint16_t position, uint16_t subDeviceID);

    void sendSetVelocityPacket(float velocity, uint16_t subDeviceID);

   public:
    ActuatorModule();
    ~ActuatorModule();

    /**
     * @brief Pushes the current state of the ODrive module to the physical module
     *
     * @return true, if the state was successfully pushed
     * @return false, if the state was not successfully pushed
     */
    bool pushState() override;
    /**
     * @brief Pulls the current state of the ODrive module from the physical module
     *
     * @return true, if the state was successfully pulled
     * @return false, if the state was not successfully pulled
     */
    bool pullState() override;
};

#endif