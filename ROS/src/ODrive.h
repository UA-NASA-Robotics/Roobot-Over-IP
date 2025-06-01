#ifndef ODRIVE_NODE_H
#define ODRIVE_NODE_H

#include "../../lib/UDP-API/oDrive.h"
#include "base.h"
#include "roi_ros/action/target_joint_state.hpp"
#include "roi_ros/msg/power.hpp"
#include "roi_ros/srv/target_joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"

class ODriveModule : public BaseModule {
   protected:
    // Msg publishers
    rclcpp::Publisher<roi_ros::msg::Power>::SharedPtr _power_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr _motor_temperature_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr _fet_temperature_publisher_;

    // Service servers
    rclcpp::Service<roi_ros::srv::TargetJointState>::SharedPtr _goto_position_service_;
    rclcpp::Service<roi_ros::srv::TargetJointState>::SharedPtr _goto_relative_position_service_;
    rclcpp::Service<roi_ros::srv::TargetJointState>::SharedPtr _set_torque_service_;
    rclcpp::Service<roi_ros::srv::TargetJointState>::SharedPtr _set_velocity_service_;

    // Action servers
    rclcpp_action::Server<roi_ros::action::TargetJointState>::SharedPtr
        _goto_position_action_server_;
    rclcpp_action::Server<roi_ros::action::TargetJointState>::SharedPtr
        _goto_relative_position_action_server_;

    // State Duplication (used for reference, and pushpull state)
    uint8_t _controlMode;
    uint8_t _inputMode;
    float _inputPosition;
    float _inputVelocity;
    float _inputTorque;

    uint32_t _errorCode;  // maybe not needed

    float _position;
    float _velocity;

    float _busVoltage;
    float _current;
    float _motorTemperature;
    float _fetTemperature;

    float _relativeStartPosition;  // used for relative position action to determine the completion

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
     * @brief Publishes the power message, vorlage and current.
     *
     */
    void publishPowerMessage();

    /**
     * @brief Publishes the state message, position and velocity.
     *
     */
    void publishStateMessage();

    /**
     * @brief Publishes the temperature message, motor and fet.
     *
     */
    void publishTemperatureMessage();

    void publishFetTemperatureMessage();
    void publishMotorTemperatureMessage();

    /**
     * @brief Callback for the goto position service
     *
     * @param request
     * @param response
     */
    void gotoPositionServiceHandler(
        const roi_ros::srv::TargetJointState::Request::SharedPtr request,
        roi_ros::srv::TargetJointState::Response::SharedPtr response);

    /**
     * @brief Callback for the goto relative position service
     *
     * @param request
     * @param response
     */
    void gotoRelativePositionServiceHandler(
        const roi_ros::srv::TargetJointState::Request::SharedPtr request,
        roi_ros::srv::TargetJointState::Response::SharedPtr response);

    /**
     * @brief Callback for the set torque service
     *
     * @param request
     * @param response
     */
    void setTorqueServiceHandler(const roi_ros::srv::TargetJointState::Request::SharedPtr request,
                                 roi_ros::srv::TargetJointState::Response::SharedPtr response);

    /**
     * @brief Callback for the set velocity service
     *
     * @param request
     * @param response
     */
    void setVelocityServiceHandler(const roi_ros::srv::TargetJointState::Request::SharedPtr request,
                                   roi_ros::srv::TargetJointState::Response::SharedPtr response);

    // Action goal handlers

    rclcpp_action::GoalResponse gotoPositionGoalHandler(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const roi_ros::action::TargetJointState::Goal> goal);

    rclcpp_action::GoalResponse gotoRelativePositionGoalHandler(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const roi_ros::action::TargetJointState::Goal> goal);

    // Action accepted handlers

    void gotoPositionAcceptedHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    void gotoRelativePositionAcceptedHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    // Action cancel handlers

    rclcpp_action::CancelResponse gotoPositionCancelHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    rclcpp_action::CancelResponse gotoRelativePositionCancelHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    // Action execution handlers

    void gotoPositionExecuteHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    void gotoRelativePositionExecuteHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    void sendGotoPositionPacket(float position, float velocity_feedforward,
                                float torque_feedforward);

    void sendGotoRelativePositionPacket(float position, float velocity_feedforward,
                                        float torque_feedforward);

    void sendSetTorquePacket(float torque);

    void sendSetVelocityPacket(float velocity, float torque_feedforward);

    std::string oDriveErrorToString(uint32_t errorCode);

    bool validateVelTorque(float velocity, float torque_feedforward);

    float revToRad(float revs);

    float radToRev(float radians);

   public:
    ODriveModule();
    ~ODriveModule();

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