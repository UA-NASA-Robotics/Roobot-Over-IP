#ifndef GENERALGPIO_H
#define GENERALGPIO_H

#include "base.h"

class ODriveModule : public BaseModule {
   protected:
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

   public:
    GeneralGPIOModule();
    ~GeneralGPIOModule();

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