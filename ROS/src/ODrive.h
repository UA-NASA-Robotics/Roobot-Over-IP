#ifndef GENERALGPIO_H
#define GENERALGPIO_H

#include "base.h"
#include "roi_ros/action/o_drive_goto_position.hpp"
#include "roi_ros/action/o_drive_goto_relative_position.hpp"
#include "roi_ros/msg/o_drive_power.hpp"
#include "roi_ros/msg/o_drive_state.hpp"
#include "roi_ros/msg/o_drive_temperature.hpp"
#include "roi_ros/srv/o_drive_goto_position.hpp"
#include "roi_ros/srv/o_drive_goto_relative_position.hpp"
#include "roi_ros/srv/o_drive_set_torque.hpp"
#include "roi_ros/srv/o_drive_set_velocity.hpp"

class ODriveModule : public BaseModule {
   protected:
    // Msg publishers
    rclcpp::Publisher<roi_ros::msg::ODrivePower>::SharedPtr powerPublisher;
    rclcpp::Publisher<roi_ros::msg::ODriveState>::SharedPtr statePublisher;
    rclcpp::Publisher<roi_ros::msg::ODriveTemperature>::SharedPtr temperaturePublisher;

    // Service servers
    rclcpp::Service<roi_ros::srv::ODriveGotoPosition>::SharedPtr gotoPositionService;
    rclcpp::Service<roi_ros::srv::ODriveGotoRelativePosition>::SharedPtr
        gotoRelativePositionService;
    rclcpp::Service<roi_ros::srv::ODriveSetTorque>::SharedPtr setTorqueService;
    rclcpp::Service<roi_ros::srv::ODriveSetVelocity>::SharedPtr setVelocityService;

    // Action servers
    rclcpp_action::Server<roi_ros::action::ODriveGotoPosition>::SharedPtr gotoPositionActionServer;
    rclcpp_action::Server<roi_ros::action::ODriveGotoRelativePosition>::SharedPtr
        gotoRelativePositionActionServer;

    /**
     * @brief A callback function for the module to handle octet parameter changes
     *
     * @param parameter
     * @return * rcl_interfaces::msg::SetParametersResult
     */
    rcl_interfaces::msg::SetParametersResult octetParameterCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    /**
     * @brief A callback function for the module to handle alias parameter changes
     *
     * @param parameter
     * @return * rcl_interfaces::msg::SetParametersResult
     */
    rcl_interfaces::msg::SetParametersResult aliasParameterCallback(
        const std::vector<rclcpp::Parameter> &parameters);

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
     * @brief Callback for the response from the sysadmin agent when a response is received
     *
     * @param response
     */
    void sysadminResponseCallback(const roi_ros::msg::SerializedPacket response);

    /**
     * @brief Implement a fuction that publishes a health update including all relevant information
     * @breif This function is utilized by the connectionState Subscription when necessary
     *
     */
    void publishHealthMessage();

    /**
     * @brief Callback for the goto position service
     *
     * @param request
     * @param response
     */
    void gotoPositionServiceHandler(
        const roi_ros::srv::ODriveGotoPosition::Request::SharedPtr request,
        roi_ros::srv::ODriveGotoPosition::Response::SharedPtr response);

    /**
     * @brief Callback for the goto relative position service
     *
     * @param request
     * @param response
     */
    void gotoRelativePositionServiceHandler(
        const roi_ros::srv::ODriveGotoRelativePosition::Request::SharedPtr request,
        roi_ros::srv::ODriveGotoRelativePosition::Response::SharedPtr response);

    /**
     * @brief Callback for the set torque service
     *
     * @param request
     * @param response
     */
    void setTorqueServiceHandler(const roi_ros::srv::ODriveSetTorque::Request::SharedPtr request,
                                 roi_ros::srv::ODriveSetTorque::Response::SharedPtr response);

    /**
     * @brief Callback for the set velocity service
     *
     * @param request
     * @param response
     */
    void setVelocityServiceHandler(
        const roi_ros::srv::ODriveSetVelocity::Request::SharedPtr request,
        roi_ros::srv::ODriveSetVelocity::Response::SharedPtr response);

    // Action goal handlers

    rclcpp_action::GoalResponse gotoPositionGoalHandler(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const roi_ros::action::ODriveGotoPosition::Goal> goal);

    rclcpp_action::GoalResponse gotoRelativePositionGoalHandler(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const roi_ros::action::ODriveGotoRelativePosition::Goal> goal);

    // Action cancel handlers

    rclcpp_action::CancelResponse gotoPositionCancelHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoPosition>>
            goalHandle);

    rclcpp_action::CancelResponse gotoRelativePositionCancelHandler(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoRelativePosition>>
            goalHandle);

    // Action execution handlers

    void gotoPositionExecuteHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoPosition>>
            goalHandle);

    void gotoRelativePositionExecuteHandler(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoRelativePosition>>
            goalHandle);

    // stored states (we need to remember the state of the module)
    bool _module_operational = false;
    uint16_t _module_state = 0;
    bool _module_error = false;
    std::string _module_error_message = "";

   public:
    ODriveModule();
    ~ODriveModule();

    /**
     * @brief Pushes the current state of the ODrive module to the physical module
     *
     * @return true, if the state was successfully pushed
     * @return false, if the state was not successfully pushed
     */
    bool pushState();
    /**
     * @brief Pulls the current state of the ODrive module from the physical module
     *
     * @return true, if the state was successfully pulled
     * @return false, if the state was not successfully pulled
     */
    bool pullState();
};

#endif