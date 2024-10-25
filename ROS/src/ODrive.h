#ifndef GENERALGPIO_H
#define GENERALGPIO_H

#include "base.h"
#include "roi_ros/msg/angular_measurement.hpp"
#include "roi_ros/msg/linear_measurement.hpp"
#include "roi_ros/srv/set_postion.hpp"
#include "roi_ros/srv/set_relative_position.hpp"
#include "roi_ros/srv/set_torque.hpp"
#include "roi_ros/srv/set_velocity.hpp"

class ODriveModule : public BaseModule {
   protected:
    // position publisher
    rclcpp::Publisher<roi_ros::msg::AngularMeasurement>::SharedPtr _positionPublisher;

    // velocity publisher
    rclcpp::Publisher<roi_ros::msg::LinearMeasurement>::SharedPtr _velocityPublisher;

    // torque publisher
    rclcpp::Publisher<roi_ros::msg::LinearMeasurement>::SharedPtr _torquePublisher;

    // bus voltage publisher
    rclcpp::Publisher<roi_ros::msg::LinearMeasurement>::SharedPtr _busVoltagePublisher;

    // current publisher
    rclcpp::Publisher<roi_ros::msg::LinearMeasurement>::SharedPtr _currentPublisher;

    // fet temperature publisher
    rclcpp::Publisher<roi_ros::msg::LinearMeasurement>::SharedPtr _fetTemperaturePublisher;

    // motor temperature publisher
    rclcpp::Publisher<roi_ros::msg::LinearMeasurement>::SharedPtr _motorTemperaturePublisher;

    // service for setting target
    rclcpp::Service<roi_ros::srv::SetPostion>::SharedPtr _setPositionService;
    rclcpp::Service<roi_ros::srv::SetRelativePosition>::SharedPtr _setRelativePositionService;
    rclcpp::Service<roi_ros::srv::SetTorque>::SharedPtr _setTorqueService;
    rclcpp::Service<roi_ros::srv::SetVelocity>::SharedPtr _setVelocityService;

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
     * @brief Callback for the set position service
     *
     * @param request
     * @param response
     */
    void setPositionServiceHandler(const roi_ros::srv::SetPostion::Request::SharedPtr request,
                                   roi_ros::srv::SetPostion::Response::SharedPtr response);

    /**
     * @brief Set the Relative Position Service Handler object
     *
     * @param request
     * @param response
     */
    void setRelativePositionServiceHandler(
        const roi_ros::srv::SetRelativePosition::Request::SharedPtr request,

        roi_ros::srv::SetRelativePosition::Response::SharedPtr response);

    /**
     * @brief Set the Torque Service Handler object
     *
     * @param request
     * @param response
     */
    void setTorqueServiceHandler(const roi_ros::srv::SetTorque::Request::SharedPtr request,
                                 roi_ros::srv::SetTorque::Response::SharedPtr response);

    /**
     * @brief Set the Velocity Service Handler object
     *
     * @param request
     * @param response
     */
    void setVelocityServiceHandler(const roi_ros::srv::SetVelocity::Request::SharedPtr request,
                                   roi_ros::srv::SetVelocity::Response::SharedPtr response);

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
     * @brief WIP, Pulls the current state of the ODrive module from the physical module
     *
     * @return true, if the state was successfully pulled
     * @return false, if the state was not successfully pulled
     */
    bool pullState();
};

#endif