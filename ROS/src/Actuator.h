#ifndef ACTUATOR_ROS_H
#define ACTUATOR_ROS_H

#include "../../lib/UDP-API/actuator.h"
#include "base.h"
#include "roi_ros/msg/duration_array.hpp"
#include "roi_ros/action/target_joint_state.hpp"
#include "roi_ros/srv/target_joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ActuatorModule : public BaseModule {
   protected:
    // Msg publishers (An arbitrary number of publishers can be created, one for each actuator)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _state_publisher_;
    rclcpp::Publisher<roi_ros::msg::DurationArray>::SharedPtr
        _home_elapsed_time_publisher_;

    // Service servers
    rclcpp::Service<roi_ros::srv::TargetJointState>::SharedPtr _goto_position_service_;
    rclcpp::Service<roi_ros::srv::TargetJointState>::SharedPtr _goto_relative_position_service_;
    rclcpp::Service<roi_ros::srv::TargetJointState>::SharedPtr _set_velocity_service_;

    // Action servers
    rclcpp_action::Server<roi_ros::action::TargetJointState>::SharedPtr
        _goto_absolute_position_action_server_;
    rclcpp_action::Server<roi_ros::action::TargetJointState>::SharedPtr
        _goto_relative_position_action_server_;

    // State Duplication (used for reference, and pushpull state)
    std::vector<uint8_t> _controlModes;
    std::vector<uint8_t> _inputPositions;
    std::vector<uint16_t> _relativeStartPositions;
    std::vector<float> _inputVelocities;

    std::vector<uint16_t> _positions;
    std::vector<float> _velocities;

    std::vector<uint32_t> _homeElapsedTimes;

    rclcpp::TimerBase::SharedPtr _parameterTimer;
    std::vector<float> _positionPIDMemory;
    std::vector<float> _velocityPIDMemory;

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
    void _publishStateMessage();

    /**
     * @brief Published the elapsed time since the actuator was homed.
     *
     */
    void _publishHomeElapsedTimeMessage();

    /**
     * @brief Callback for the goto position service
     *
     * @param request
     * @param response
     */
    void _goto_position_service_callback_(
        const roi_ros::srv::TargetJointState::Request::SharedPtr request,
        roi_ros::srv::TargetJointState::Response::SharedPtr response);

    /**
     * @brief Callback for the goto relative position service
     *
     * @param request
     * @param response
     */
    void _goto_relative_position_service_callback_(
        const roi_ros::srv::TargetJointState::Request::SharedPtr request,
        roi_ros::srv::TargetJointState::Response::SharedPtr response);

    /**
     * @brief Callback for the set velocity service
     *
     * @param request
     * @param response
     */
    void _set_velocity_service_callback_(
        const roi_ros::srv::TargetJointState::Request::SharedPtr request,
        roi_ros::srv::TargetJointState::Response::SharedPtr response);

    /**
     * @brief Action goal handler for goto position action
     *
     * @param uuid
     * @param goal
     * @return rclcpp_action::GoalResponse
     */
    rclcpp_action::GoalResponse _goto_position_goal_handler_(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const roi_ros::action::TargetJointState::Goal> goal);

    /**
     * @brief Action goal handler for goto relative position action
     *
     * @param uuid
     * @param goal
     * @return rclcpp_action::GoalResponse
     */
    rclcpp_action::GoalResponse _goto_relative_position_goal_handler_(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const roi_ros::action::TargetJointState::Goal> goal);

    // Action accepted handlers

    /**
     * @brief Action accepted handler for goto position action
     *
     * @param goalHandle
     */
    void _goto_position_accepted_handler_(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    /**
     * @brief Action accepted handler for goto relative position action
     *
     * @param goalHandle
     */
    void _goto_relative_position_accepted_handler_(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    // Action cancel handlers

    /**
     * @brief Action cancel handler for goto position action
     *
     * @param goalHandle
     */
    rclcpp_action::CancelResponse _goto_position_cancel_handler_(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    /**
     * @brief Action cancel handler for goto relative position action
     *
     * @param goalHandle
     */
    rclcpp_action::CancelResponse _goto_relative_position_cancel_handler_(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    // Action execution handlers

    /**
     * @brief Action execution handler for goto position action
     *
     * @param goalHandle
     */
    void _goto_position_execute_handler_(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);
    /**
     * @brief Action execution handler for goto relative position action
     *
     * @param goalHandle
     */
    void _goto_relative_position_execute_handler_(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::TargetJointState>>
            goalHandle);

    /**
     * @brief Sends packets necesary to set absolute position on the actuator
     *
     * @param position mm, the absolute position to set
     * @param velocity_feedforward mm/s, the velocity feedforward to set
     * @param subDeviceID
     */
    void sendGotoAbsolutePositionPacket(uint16_t position, float velocity_feedforward,
                                        uint16_t subDeviceID);

    /**
     * @brief Sends packets necesary to set relative position on the actuator
     *
     * @param position mm, the relative position to set
     * @param velocity_feedforward mm/s, the velocity feedforward to set
     * @param subDeviceID
     */
    void sendGotoRelativePositionPacket(uint16_t position, float velocity_feedforward,
                                        uint16_t subDeviceID);

    /**
     * @brief Sends packets necessary to set velocity mode on the actuator
     *
     * @param velocity mm/s, the velocity to set
     * @param subDeviceID
     */
    void sendSetVelocityPacket(float velocity, uint16_t subDeviceID);

    /**
     * @brief Sends packets necessary to update velocity PID controller
     *
     *
     */
    void sendSetVelocityPIDPacket(float p, float i, float d, uint16_t subDeviceID);

    /**
     * @brief Sends packets necessary to set the position PID controller
     *
     */
    void sendSetPositionPIDPacket(float p, float i, float d, uint16_t subDeviceID);

    /**
     * @brief Initializes all actuator module publishers.
     *
     */
    void initializeTopics();

    /**
     * @brief Validates the velocity and torque feedforward values
     *
     * @param position m, the position to validate
     * @param velocity m/s, the velocity feedforward to validate
     * @return true if the values are valid
     * @return false if the values are invalid
     */
    bool validateInput(float position, float velocity, uint16_t subDeviceID);

    /**
     * @brief Checks for change in actuator parameters, like the one which checks for octet
     * parameter changes
     *
     * Must be called by a repeating timer to ensure that the parameters are checked. Done in
     * constructor.
     *
     */
    virtual void actuatorParameterCheck();

   public:
    ActuatorModule();

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
