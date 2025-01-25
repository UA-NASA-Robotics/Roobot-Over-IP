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
    uint8_t controlMode;
    uint8_t inputMode;

    float desiredPosition;
    float desiredVelocity;
    float desiredTorque;

    bool paused;

    statusManager::statusManager& moduleStatusManager;

    long baudrate;
    ODriveUART odrive;
    SoftwareSerial odrive_serial;

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