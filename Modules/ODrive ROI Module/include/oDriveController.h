#ifndef ODRIVECONTROLLER_H
#define ODRIVECONTROLLER_H

#include <ODriveUART.h>

#include "../../../lib/Packet.h"
#include "../../../lib/UDP-API/oDrive.h"
#include "../../../lib/floatCast.h"
#include "../../../lib/moduleLib/statusManager.h"
#include "oDriveError.h"
#if ODRIVE_MODULE_REV < 3
#include <SoftwareSerial.h>
#elif ODRIVE_MODULE_REV == 3
#include <HardwareSerial.h>
#endif

class ODriveController {
   private:
    // Current odrive state:
    uint8_t controlMode;
    uint8_t inputMode;

    float position;
    float velocity;
    float torque;

    bool paused;
    bool userDisabled;

    statusManager::statusManager& moduleStatusManager;

    long baudrate;
    
    #if ODRIVE_MODULE_REV < 3
    SoftwareSerial odrive_serial;
    #else
    HardwareSerial odrive_serial;
    #endif

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
     * nothing. Will not resume if the user has disabled the motor via disable().
     *
     */
    void resume();

    /**
     * @brief Disables the motor by setting the ODrive to idle state (user-commanded).
     * Unlike pause(), this is triggered by an explicit user command, not a watchdog event.
     * The motor will not re-enable on watchdog resume until enable() is called.
     *
     */
    void disable();

    /**
     * @brief Re-enables the motor by setting the ODrive to closed-loop control (user-commanded).
     * Clears the user-disabled state. Will not enable if the watchdog has paused the motor.
     *
     */
    void enable();

    /**
     * @brief Returns whether the motor is currently enabled (not user-disabled and not paused)
     *
     * @return true if motor is in closed-loop control
     */
    bool isEnabled();

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