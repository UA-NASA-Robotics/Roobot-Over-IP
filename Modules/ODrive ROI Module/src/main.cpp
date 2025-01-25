#include <Arduino.h>
#include <ODriveUART.h>
#include <SoftwareSerial.h>
#include <stdint.h>

// Define the default debug mode for the ROI module
#ifndef DEBUG
#define DEBUG false
#endif
// Modify debug mode in "PlatformIO.ini" file, NOT here

#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/floatCast.h"
#include "../../../lib/moduleLib/infrastructure.h"
#include "oDriveError.h"

uint8_t* generalBuffer(nullptr);  // Sharing a large buffer from the infrastructure in this main.cpp
ModuleInfrastructure* infraRef(
    nullptr);  // Reference to the infrastructure for withing handleGeneralPacket function

/**
 * @brief Set the Control and Input mode of the ODrive
 *
 * @param controlMode , the control mode to set the ODrive to (ROI VALUE)
 * @param inputMode , the input mode to set the ODrive to (ROI VALUE)
 * @return true, if the control and input mode were set successfully
 * @return false, if the control and input mode were not set successfully
 */
bool setControlInputMode(uint8_t controlMode, uint8_t inputMode) {
    // Set the ODrive control mode based on the parameter
    uint8_t ODriveControlModeVal = 0;
    switch (controlMode) {
        case ODriveConstants::POSITIONMODE:
            ODriveControlModeVal = CONTROL_MODE_POSITION_CONTROL;
            break;
        case ODriveConstants::VELOCITYMODE:
            ODriveControlModeVal = CONTROL_MODE_VELOCITY_CONTROL;
            break;
        case ODriveConstants::TORQUEMODE:
            ODriveControlModeVal = CONTROL_MODE_TORQUE_CONTROL;
            break;
        default:
            break;
    };

    // Set the ODrive input mode based on the parameter
    uint8_t ODriveInputModeVal = 0;
    switch (inputMode) {
        case ODriveConstants::TRAP_TRAJ_MODE:
            ODriveInputModeVal = INPUT_MODE_TRAP_TRAJ;
            break;
        case ODriveConstants::POS_FILTER_MODE:
            ODriveInputModeVal = INPUT_MODE_POS_FILTER;
            break;
        case ODriveConstants::VELOCITY_RAMP_MODE:
            ODriveInputModeVal = INPUT_MODE_VEL_RAMP;
            break;
        case ODriveConstants::TORQUE_RAMP_MODE:
            ODriveInputModeVal = INPUT_MODE_TORQUE_RAMP;
            break;
        case ODriveConstants::AUTO_BEST_FIT_MODE:
            switch (controlMode) {
                case ODriveConstants::POSITIONMODE:
                    ODriveInputModeVal = INPUT_MODE_TRAP_TRAJ;
                    break;
                case ODriveConstants::VELOCITYMODE:
                    ODriveInputModeVal = INPUT_MODE_VEL_RAMP;
                    break;
                case ODriveConstants::TORQUEMODE:
                    ODriveInputModeVal = INPUT_MODE_TORQUE_RAMP;
                    break;
                default:
                    break;
            };
    };

    // issue the commands to the ODrive
    // odrive.setState(AXIS_STATE_IDLE);
    odrive.setParameter(F("axis0.controller.config.control_mode"), ODriveControlModeVal);
    odrive.setParameter(F("axis0.controller.config.input_mode"), ODriveInputModeVal);
    // odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);

    return true;  // no error handling yet
}

/**
 * @brief Sends the desired position/velocity/torque to the ODrive. It applies feed forwards
 * when applicable
 *
 * @return true, if the desired position/velocity/torque was sent successfully
 * @return false, if the desired position/velocity/torque was not sent successfully
 */
bool applyFeeds() {
    switch (oDriveControlMode) {
        case ODriveConstants::POSITIONMODE: {
            if (oDriveInputMode == ODriveConstants::TRAP_TRAJ_MODE) {
                odrive.trapezoidalMove(desiredPosition);
            } else
                odrive.setPosition(desiredPosition, desiredVelocity, desiredTorque);
            break;
        }

        case ODriveConstants::VELOCITYMODE:
            odrive.setVelocity(desiredVelocity, desiredTorque);
            break;

        case ODriveConstants::TORQUEMODE:
            odrive.setTorque(desiredTorque);
            break;

        default:
            return false;
            break;
    }

    return true;
}

// Function to handle a general packet
//@param packet The packet to handle
ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet packet) {
    uint16_t action = packet.getActionCode();  // Get the action code from the packet
    // uint16_t subDeviceID = packet.getSubDeviceID();  // Get the subdevice ID from the packet
    packet.getData(generalBuffer,
                   ROIConstants::ROIMAXPACKETPAYLOAD);  // Get the payload from the packet

    ROIPackets::Packet replyPacket = packet.swapReply();  // Create a reply packet

    if (!(action &
          ODriveConstants::MaskConstants::GETMASK)) {  // Split the code into setters and getters
        infraRef->moduleStatusManager
            .notifySystemConfigured();  // Notify the status manager that
                                        // the system has been configured, removes the blank state
        switch (action &
                (!ODriveConstants::MaskConstants::SETMASK)) {  // remove the set mask (note setmask
                                                               // = 0 atm) function does not modify
                                                               // the action code, but good practice
                                                               // incase setmask changes

            case ODriveConstants::MaskConstants::ControlMode:
                oDriveControlMode = generalBuffer[0];  // Set the control mode of the ODrive
                setControlInputMode(oDriveControlMode, oDriveInputMode);
#if DEBUG
                Serial.print(F("Control Mode Set:"));
                Serial.println(generalBuffer[0]);
#endif
                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::InputMode:
                oDriveInputMode = generalBuffer[0];  // Set the input mode of the ODrive
                setControlInputMode(oDriveControlMode, oDriveInputMode);

#if DEBUG
                Serial.print(F("Input Mode Set:"));
                Serial.println(generalBuffer[0]);
#endif

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

ModuleInfrastructure infra(10, 2, moduleTypesConstants::ODrive,
                           handleGeneralPacket);  // Create an instance of the infrastructure

void setup() {
    infra.init();  // Initialize the infrastructure (also defines Serial)

    infraRef = &infra;  // lets the handleGeneralPacket function access the infrastructure
    generalBuffer =
        &infra.generalBuffer[0];  // lets the handleGeneralPacket function access the buffer

    infra.moduleStatusManager.notifyInitializedStatus();  // Notify the infrastructure that the
                                                          // module has been initialized.
}

ISR(TIMER1_OVF_vect) {
    // This ISR is called every 1.048 seconds by timer1 overflow
    infra.interruptNotification();  // Notify the infrastructure of the interrupt
}

void loop() {
    // Check for ODrive connection
    if (odrive.getState() == AXIS_STATE_UNDEFINED) {
#if DEBUG
        Serial.println(F("ODrive is not connected. Reinitalizing."));
        delay(1000);  // delay for 1 second for serial to print
#endif
        infra.resetFunction();  // The reset function is called to restart the module
        // The program will not fully resume operation until the ODrive is connected
    }

    uint32_t odriveError = odrive.getParameterAsInt(F("axis0.active_errors"));
    if (odriveError != ODriveConstants::ODRIVE_ERROR_NONE ||
        odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
#if DEBUG
        Serial.print(F("ODrive Error: "));
        Serial.println(odriveError);
#endif

        infra.moduleStatusManager.notifySystemError(!oDriveError::errorIsOperable(odriveError));
        if (oDriveError::errorShouldAutoClear(odriveError)) {
#if DEBUG
            Serial.println(F("Auto Clearing Error"));
#endif
            odrive.clearErrors();
            odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
    } else {
        infra.moduleStatusManager.notifyClearError();
    }

    infra.tick();  // Tick the infrastructure
}
