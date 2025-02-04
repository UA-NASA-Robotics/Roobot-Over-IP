#ifndef ODRIVECONTROLLER_H
#define ODRIVECONTROLLER_H

#include <ODriveUART.h>
#include <SoftwareSerial.h>

#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/floatCast.h"
#include "../../../lib/moduleLib/statusManager.h"
#include "oDriveError.h"

class ODriveController {
   private:
    // Current odrive state:
    uint8_t controlMode;
    uint8_t inputMode;

    float position;
    float velocity;
    float torque;

    bool paused;

    statusManager::statusManager& moduleStatusManager;

    long baudrate;
    SoftwareSerial odrive_serial;
    ODriveUART odrive;

    void applyFeeds();                     // apply all the feeds to the ODrive
    void applyFeeds(uint8_t controlMode);  // apply the feeds to the ODrive based on the control
                                           // mode
    void applyFeeds(uint8_t controlMode, uint8_t inputMode);  // apply the feeds to the ODrive based
                                                              // on the control mode and input mode
    void applyFeeds(
        float autoBestFit);  // apply the feeds to the ODrive based on the auto best fit mode
    void applyFeeds(
        float position, float velocity,
        float torque);  // apply the feeds to the ODrive based on the position, velocity, and torque
    void applyFeeds(float position, float velocity, float torque, uint8_t controlMode,
                    uint8_t inputMode);  // apply the feeds to the ODrive based on the position,
                                         // velocity, torque, control mode, and input mode

    uint8_t controlModetoEnum(uint8_t controlMode);  // convert the control mode to an enum
    uint8_t inputModetoEnum(uint8_t inputMode);      // convert the input mode to an enum

   public:
    ODriveController(uint8_t rx, uint8_t tx, long baudrate,
                     statusManager::statusManager& moduleStatusManager);

    /**
     * @brief Inits the ODriveController Serial Interface
     *
     */
    void init();

    /**
     * @brief Sets the Odrive to the idle state, used if connection is lost
     *
     */
    void pause();

    /**
     * @brief Sets the Odrive to the closed loop control state, if was paused. If not paused, does
     * nothing
     *
     */
    void resume();

    /**
     * @brief Tries to clear any errors on the ODrive and set it to a non-moving on state
     *
     */
    void reset();

    /**
     * @brief Used to detect and handle any errors on the ODrive
     *
     */
    void tick();

    /**
     * @brief Handles the general packet from the ROI, and sends the appropriate commands to the
     * ODrive. NOTE This does not check matching subdevice IDs
     *
     * @param packet , the packet to handle
     * @return ROIPackets::Packet, the reply packet
     */
    ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet& packet);
};

#endif