#include "../include/oDriveController.h"

// PRIVATE FUNCTIONS

void ODriveController::applyFeeds() {
    applyFeeds(controlMode, inputMode, position, velocity, torque);
}

void ODriveController::applyFeeds(uint8_t controlMode) {
    odrive.setParameter(F("axis0.controller.config.control_mode"), controlModetoEnum(controlMode));
}

void ODriveController::applyFeeds(uint8_t controlMode, uint8_t inputMode) {
    applyFeeds(controlMode);
    odrive.setParameter(F("axis0.controller.config.input_mode"), inputModetoEnum(inputMode));
}

void ODriveController::applyFeeds(float autoBestFit) {
    switch (controlMode) {
        case ODriveConstants::POSITIONMODE:
            odrive.setPosition(autoBestFit, velocity, torque);
            break;
        case ODriveConstants::VELOCITYMODE:
            odrive.setVelocity(autoBestFit, torque);
            break;
        case ODriveConstants::TORQUEMODE:
            odrive.setTorque(autoBestFit);
            break;
        default:
            break;
    }
}

void ODriveController::applyFeeds(float position, float velocity, float torque) {
    switch (controlMode) {
        case ODriveConstants::POSITIONMODE:
            odrive.setPosition(position, velocity, torque);
            break;
        case ODriveConstants::VELOCITYMODE:
            odrive.setVelocity(velocity, torque);
            break;
        case ODriveConstants::TORQUEMODE:
            odrive.setTorque(torque);
            break;
        default:
            break;
    }
}

void ODriveController::applyFeeds(float position, float velocity, float torque, uint8_t controlMode,
                                  uint8_t inputMode) {
    applyFeeds(controlMode, inputMode);
    applyFeeds(position, velocity, torque);
}

uint8_t ODriveController::controlModetoEnum(uint8_t controlMode) {
    switch (controlMode) {
        case ODriveConstants::POSITIONMODE:
            return CONTROL_MODE_POSITION_CONTROL;
        case ODriveConstants::VELOCITYMODE:
            return CONTROL_MODE_VELOCITY_CONTROL;
        case ODriveConstants::TORQUEMODE:
            return CONTROL_MODE_TORQUE_CONTROL;
        default:
            return CONTROL_MODE_POSITION_CONTROL;
    }
}

uint8_t ODriveController::inputModetoEnum(uint8_t inputMode) {
    switch (inputMode) {
        case ODriveConstants::TRAP_TRAJ_MODE:
            return INPUT_MODE_TRAP_TRAJ;
        case ODriveConstants::POS_FILTER_MODE:
            return INPUT_MODE_POS_FILTER;
        case ODriveConstants::VELOCITY_RAMP_MODE:
            return INPUT_MODE_VEL_RAMP;
        case ODriveConstants::TORQUE_RAMP_MODE:
            return INPUT_MODE_TORQUE_RAMP;
        case ODriveConstants::AUTO_BEST_FIT_MODE:
            switch (controlMode) {
                case ODriveConstants::POSITIONMODE:
                    return INPUT_MODE_TRAP_TRAJ;
                case ODriveConstants::VELOCITYMODE:
                    return INPUT_MODE_VEL_RAMP;
                case ODriveConstants::TORQUEMODE:
                    return INPUT_MODE_TORQUE_RAMP;
                default:
                    return INPUT_MODE_TRAP_TRAJ;
            }
        default:
            return INPUT_MODE_TRAP_TRAJ;
    }
}

// PUBLIC FUNCTIONS

ODriveController::ODriveController(uint8_t rx, uint8_t tx, long baudrate,
                                   statusManager::statusManager& moduleStatusManager)
    : odrive_serial(rx, tx),
      odrive(odrive_serial),
      controlMode(ODriveConstants::POSITIONMODE),
      inputMode(ODriveConstants::AUTO_BEST_FIT_MODE),
      position(0),
      velocity(0),
      torque(0),
      baudrate(baudrate),
      moduleStatusManager(moduleStatusManager),
      paused(false) {}

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

void ODriveController::reset() {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void ODriveController::tick() {
    if (odrive.getState() == AXIS_STATE_UNDEFINED) {  // Check for ODrive connection
#if DEBUG
        Serial.println(F("ODrive disconnected."));
        delay(1000);  // delay for 1 second for serial to print
#endif
        moduleStatusManager.notifySystemError(true);
        // The program will not fully resume operation until the ODrive is connected
    }

    uint32_t odriveError = odrive.getParameterAsInt(F("axis0.active_errors"));
    if (odriveError != ODriveConstants::ODRIVE_ERROR_NONE ||
        (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL && !paused)) {
#if DEBUG
        Serial.print(F("ODrive Error Code: "));
        Serial.println(odriveError);
#endif

        moduleStatusManager.notifySystemError(!oDriveError::errorIsOperable(odriveError));
        if (oDriveError::errorShouldAutoClear(odriveError)) {
#if DEBUG
            Serial.println(F("Auto Clearing Error"));
#endif
            odrive.clearErrors();
            odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
    } else {
        moduleStatusManager.notifyClearError();
    }
}

ROIPackets::Packet ODriveController::handleGeneralPacket(ROIPackets::Packet& packet) {
    uint16_t action = packet.getActionCode();  // Get the action code from the packet

    uint8_t generalBuffer[30];  // Create a buffer to store the data from the packet
    // uint16_t subDeviceID = packet.getSubDeviceID();  // Get the subdevice ID from the packet
    packet.getData(generalBuffer,
                   ROIConstants::ROIMAXPACKETPAYLOAD);  // Get the payload from the packet

    ROIPackets::Packet replyPacket = packet.swapReply();  // Create a reply packet

    if (!(action &
          ODriveConstants::MaskConstants::GETMASK)) {  // Split the code into setters and getters
        moduleStatusManager
            .notifySystemConfigured();  // Notify the status manager that
                                        // the system has been configured, removes the blank state
        switch (action &
                (!ODriveConstants::MaskConstants::SETMASK)) {  // remove the set mask (note setmask
                                                               // = 0 atm) function does not modify
                                                               // the action code, but good practice
                                                               // incase setmask changes

            case ODriveConstants::MaskConstants::ControlMode:
                controlMode = generalBuffer[0];  // Set the control mode of the ODrive
                applyFeeds(controlMode);

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::InputMode:
                inputMode = generalBuffer[0];  // Set the input mode of the ODrive
                applyFeeds(controlMode, inputMode);

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::Torque:
                desiredTorque =
                    floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
                applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
                Serial.print(F("Torque Set:"));
                Serial.println(desiredTorque);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::PositionSetPoint:
                desiredPosition =
                    floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
                applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
                Serial.print(F("Position Set:"));
                Serial.println(desiredPosition);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::VelocitySetPoint:
                desiredVelocity =
                    floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
                applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
                Serial.print(F("Velocity Set:"));
                Serial.println(desiredVelocity);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::PositionRelative:
                desiredPosition +=
                    floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
                applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
                Serial.print(F("Relative Position Set:"));
                Serial.println(desiredPosition);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::Error:
                odrive.clearErrors();
                infraRef->moduleStatusManager.notifyClearError();  // Notify the status manager that
                                                                   // the module has cleared errors

#if DEBUG
                Serial.println(F("Errors Cleared"));
#endif

                replyPacket.setData(1);  // return 1 for success

                break;

            default:
                Serial.print(F("Unknown Action: "));
                Serial.println(action);
                break;
        }
    } else {                                                            // getters
        switch (action & (!ODriveConstants::MaskConstants::GETMASK)) {  // remove the get mask from
                                                                        // the action code
            case ODriveConstants::MaskConstants::ControlMode:
                replyPacket.setData(oDriveControlMode);
                break;

            case ODriveConstants::MaskConstants::InputMode:
                replyPacket.setData(oDriveInputMode);
                break;

            case ODriveConstants::MaskConstants::Torque: {
                floatCast::floatToUint8Array(desiredTorque, generalBuffer, 0, 3);

                replyPacket.setData(generalBuffer, 4);  // Set the data in the reply packet
            }

            case ODriveConstants::MaskConstants::PositionSetPoint: {
                floatCast::floatToUint8Array(desiredPosition, generalBuffer, 0, 3);

                replyPacket.setData(generalBuffer, 4);  // Set the data in the reply packet

                break;
            }

            case ODriveConstants::MaskConstants::VelocitySetPoint: {
                floatCast::floatToUint8Array(desiredVelocity, generalBuffer, 0, 3);

                replyPacket.setData(generalBuffer, 4);  // Set the data in the reply packet

                break;

                case ODriveConstants::MaskConstants::Position: {
                    float pos = odrive.getPosition();
                    floatCast::floatToUint8Array(pos, generalBuffer, 0, 3);

                    replyPacket.setData(generalBuffer,
                                        4);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::Velocity: {
                    float vel = odrive.getVelocity();
                    floatCast::floatToUint8Array(vel, generalBuffer, 0, 3);

                    replyPacket.setData(generalBuffer,
                                        4);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::BusVoltage: {
                    float busVoltage = odrive.getParameterAsFloat(F("vbus_voltage"));
                    floatCast::floatToUint8Array(busVoltage, generalBuffer, 0, 3);

                    replyPacket.setData(generalBuffer,
                                        4);  // Set the data in the reply packet
                    break;
                }

                case ODriveConstants::MaskConstants::Current: {
                    float current = odrive.getParameterAsFloat(F("ibus"));
                    floatCast::floatToUint8Array(current, generalBuffer, 0, 3);

                    replyPacket.setData(generalBuffer,
                                        4);  // Set the data in the reply packet
                    break;
                }

                case ODriveConstants::MaskConstants::FETTemperature: {
                    float fetTemp =
                        odrive.getParameterAsFloat(F("axis0.motor.fet_thermistor.temperature"));
                    floatCast::floatToUint8Array(fetTemp, generalBuffer, 0, 3);

                    replyPacket.setData(generalBuffer,
                                        4);  // Set the data in the reply packet
                    break;
                }

                case ODriveConstants::MaskConstants::MotorTemperature: {
                    float motorTemp =
                        odrive.getParameterAsFloat(F("axis0.motor.motor_thermistor.temperature"));
                    floatCast::floatToUint8Array(motorTemp, generalBuffer, 0, 3);

                    replyPacket.setData(generalBuffer,
                                        4);  // Set the data in the reply packet
                    break;
                }

                case ODriveConstants::MaskConstants::Error: {
                    uint32_t error = odrive.getParameterAsInt(F("axis0.active_errors"));

                    replyPacket.setData((error >> 24) & 0xFF, (error >> 16) & 0xFF,
                                        (error >> 8) & 0xFF,
                                        error & 0xFF);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::KinematicFeedback: {
                    ODriveFeedback feedback = odrive.getFeedback();

                    floatCast::floatToUint8Array(feedback.pos, generalBuffer, 0, 3);
                    floatCast::floatToUint8Array(feedback.vel, generalBuffer, 4, 7);

                    replyPacket.setData(generalBuffer,
                                        8);  // Set the data in the reply packet
                    break;
                }

                case ODriveConstants::MaskConstants::all: {
                    ODriveFeedback feedback = odrive.getFeedback();

                    float voltage = odrive.getParameterAsFloat(F("vbus_voltage"));
                    float current = odrive.getParameterAsFloat(F("ibus"));

                    float fetTemp =
                        odrive.getParameterAsFloat(F("axis0.motor.fet_thermistor.temperature"));
                    float motorTemp =
                        odrive.getParameterAsFloat(F("axis0.motor.motor_thermistor.temperature"));

                    uint32_t error = odrive.getParameterAsInt(F("axis0.active_errors"));

                    floatCast::floatToUint8Array(feedback.pos, generalBuffer, 0, 3);
                    floatCast::floatToUint8Array(feedback.vel, generalBuffer, 4, 7);
                    floatCast::floatToUint8Array(voltage, generalBuffer, 8, 11);
                    floatCast::floatToUint8Array(current, generalBuffer, 12, 15);
                    floatCast::floatToUint8Array(fetTemp, generalBuffer, 16, 19);
                    floatCast::floatToUint8Array(motorTemp, generalBuffer, 20, 23);

                    generalBuffer[24] = (error >> 24) & 0xFF;
                    generalBuffer[25] = (error >> 16) & 0xFF;
                    generalBuffer[26] = (error >> 8) & 0xFF;
                    generalBuffer[27] = error & 0xFF;

                    replyPacket.setData(generalBuffer,
                                        28);  // Set the data in the reply packet
                    break;
                }
            }
        }
    }
    return replyPacket;  // Return the reply packet
}