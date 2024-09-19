#ifndef GENERALGPIO_H
#define GENERALGPIO_H

#include "base.h"

class generalGPIOModule : public BaseModule {
   protected:
    uint8_t subDeviceState[GeneralGPIOConstants::COUNT] = {GeneralGPIOConstants::INPUT_MODE};
    uint8_t subDeviceMode[GeneralGPIOConstants::COUNT] = {0};

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

   public:
    generalGPIOModule();
    ~generalGPIOModule();

    bool pushState();
    bool pullState();
};

#endif