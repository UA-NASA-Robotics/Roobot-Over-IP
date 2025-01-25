#include "../include/oDriveController.h"

ODriveController::ODriveController(uint8_t rx, uint8_t tx, long baudrate,
                                   statusManager::statusManager& moduleStatusManager)
    : odrive_serial(rx, tx),
      odrive(odrive_serial),
      controlMode(ODriveConstants::POSITIONMODE),
      inputMode(ODriveConstants::AUTO_BEST_FIT_MODE),
      desiredPosition(0),
      desiredVelocity(0),
      desiredTorque(0),
      paused(false),
{
    this->baudrate = baudrate;
    this->moduleStatusManager = moduleStatusManager;
}

void ODriveController::init() {
    odrive_serial.begin(baudrate);
    delay(100);

#if DEBUG
    Serial.println(F("Waiting for ODrive..."));
#endif
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {  // Wait for the ODrive to connect
        delay(100);
    }

#if DEBUG
    Serial.println(F("Enabling closed loop control..."));
#endif
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void ODriveController::pause() {
    if (!paused) {
        odrive.setState(AXIS_STATE_IDLE);
        paused = true;
    }
}

void ODriveController::resume() {
    if (paused) {
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        paused = false;
    }
}