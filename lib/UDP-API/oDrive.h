#ifndef ODRV_API_H
#define ODRV_API_H

#include "packetTypes.h"

namespace ODriveConstants {
namespace MaskConstants {
constexpr uint8_t GETMASK = 0b10000000;  // Mask to get the value from the ODrive
constexpr uint8_t SETMASK = 0b00000000;  // Mask to set the value to the ODrive

constexpr uint8_t ControlMode = 0;       // Mask reference for control mode
constexpr uint8_t InputMode = 1;         // Mask reference for input mode
constexpr uint8_t Torque = 2;            // Mask reference for torque
constexpr uint8_t PositionSetPoint = 3;  // Mask reference for position setpoint
constexpr uint8_t VelocitySetPoint = 4;  // Mask reference for velocity setpoint
constexpr uint8_t PositionRelative = 5;  // Mask reference for relative position (set only)
constexpr uint8_t Error = 6;             // Mask reference for error

constexpr uint8_t Position = 7;  // Mask reference for position (get only)
constexpr uint8_t Velocity = 8;  // Mask reference for velocity (get only)

constexpr uint8_t BusVoltage = 9;         // Mask reference for bus voltage (get only)
constexpr uint8_t Current = 10;           // Mask reference for current (get only)
constexpr uint8_t FETTemperature = 11;    // Mask reference for fet temperature (get only)
constexpr uint8_t MotorTemperature = 12;  // Mask reference for motor temperature (get only)

constexpr uint8_t KinematicFeedback =
    13;  // Mask reference for kinematic feedback (pos and vel) (get only)

constexpr uint8_t all = 100;  // Mask reference for all values (get only)
}  // namespace MaskConstants

/*--------- Action Codes ----------------*/
constexpr actionConstant SET_CONTROL_MODE =
    MaskConstants::SETMASK |
    MaskConstants::ControlMode;  // Set the control mode of the ODrive (Position, Velocity, etc) atm
                                 // the odrive module exclusively supports closed loop control
constexpr actionConstant SET_INPUT_MODE =
    MaskConstants::SETMASK |
    MaskConstants::InputMode;  // Set the input mode of the ODrive (Position, Velocity, etc)
constexpr actionConstant SET_TORQUE =
    MaskConstants::SETMASK |
    MaskConstants::Torque;  // Set the torque (torque mode) or maximum torque of the ODrive
constexpr actionConstant SET_POSITION =
    MaskConstants::SETMASK |
    MaskConstants::PositionSetPoint;  // Set the position(position mode) of the ODrive
constexpr actionConstant SET_RELATIVE_POSITION =
    MaskConstants::SETMASK |
    MaskConstants::PositionRelative;  // Set the relative position(position mode) of the ODrive
constexpr actionConstant SET_VELOCITY =
    MaskConstants::SETMASK | MaskConstants::VelocitySetPoint;  // Set the velocity(velocity mode) or
                                                               // maximum velocity of the ODrive

constexpr actionConstant GET_CONTROL_MODE =
    MaskConstants::GETMASK | MaskConstants::ControlMode;  // Get the control mode of the ODrive
constexpr actionConstant GET_INPUT_MODE =
    MaskConstants::GETMASK | MaskConstants::InputMode;  // Get the input mode of the ODrive
constexpr actionConstant GET_TORQUE_SETPOINT =
    MaskConstants::GETMASK | MaskConstants::Torque;  // Get the torque set point of the ODrive
constexpr actionConstant GET_POSITION_SETPOINT =
    MaskConstants::GETMASK |
    MaskConstants::PositionSetPoint;  // Get the position set point of the ODrive
constexpr actionConstant GET_VELOCITY_SETPOINT =
    MaskConstants::GETMASK |
    MaskConstants::VelocitySetPoint;  // Get the velocity set point of the ODrive

constexpr actionConstant CLEAR_ERRORS =
    MaskConstants::SETMASK | MaskConstants::Error;  // Clear the errors of the ODrive (can reset the
                                                    // ODrive when in a non-operational state)
constexpr actionConstant GET_ERROR =
    MaskConstants::GETMASK | MaskConstants::Error;  // Get the errors of the ODrive

constexpr actionConstant GET_POSITION =
    MaskConstants::GETMASK | MaskConstants::Position;  // Get the position of the ODrive
constexpr actionConstant GET_VELOCITY =
    MaskConstants::GETMASK | MaskConstants::Velocity;  // Get the velocity of the ODrive
constexpr actionConstant GET_BUS_VOLTAGE =
    MaskConstants::GETMASK | MaskConstants::BusVoltage;  // Get the bus voltage of the ODrive
constexpr actionConstant GET_CURRENT =
    MaskConstants::GETMASK | MaskConstants::Current;  // Get the current of the ODrive
constexpr actionConstant GET_FET_TEMPERATURE =
    MaskConstants::GETMASK &
    MaskConstants::FETTemperature;  // Get the temperature of the ODrive transistors
constexpr actionConstant GET_MOTOR_TEMPERATURE =
    MaskConstants::GETMASK |
    MaskConstants::MotorTemperature;  // Get the temperature of the ODrive motor

constexpr actionConstant GET_KINEMATIC_FEEDBACK =
    MaskConstants::GETMASK |
    MaskConstants::KinematicFeedback;  // Get the kinematic feedback of the ODrive

constexpr actionConstant GET_ALL =
    MaskConstants::GETMASK | MaskConstants::all;  // Get all values of the ODrive

//------ Set Control Mode Constants ------
constexpr payloadConstant POSITION_MODE = 0b00000000;  // Position mode
constexpr payloadConstant VELOCITY_MODE = 0b00000001;  // Velocity mode
constexpr payloadConstant TORQUE_MODE = 0b00000010;    // Current mode

//------ Set Input Mode Constants ------
constexpr payloadConstant TRAP_TRAJ_MODE = 0b00000000;      // Trapezoidal trajectory mode
constexpr payloadConstant POS_FILTER_MODE = 0b00000001;     // Position filter mode
constexpr payloadConstant VELOCITY_RAMP_MODE = 0b00000010;  // Velocity ramp mode
constexpr payloadConstant TORQUE_RAMP_MODE = 0b00000011;    // Torque ramp mode
constexpr payloadConstant AUTO_BEST_FIT_MODE = 0b00000100;  // Auto best fit mode

//------ Error Codes ------ (Arrives as 4 big-endian bytes)
constexpr long ODRIVE_ERROR_NONE = 0x00000000;
constexpr long ODRIVE_ERROR_INITIALIZING = 0x00000001;
constexpr long ODRIVE_ERROR_SYSTEM_LEVEL = 0x00000002;
constexpr long ODRIVE_ERROR_TIMING_ERROR = 0x00000004;
constexpr long ODRIVE_ERROR_MISSING_ESTIMATE = 0x00000008;
constexpr long ODRIVE_ERROR_BAD_CONFIG = 0x00000010;
constexpr long ODRIVE_ERROR_DRV_FAULT = 0x00000020;
constexpr long ODRIVE_ERROR_MISSING_INPUT = 0x00000040;
constexpr long ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE = 0x00000100;
constexpr long ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE = 0x00000200;
constexpr long ODRIVE_ERROR_DC_BUS_OVER_CURRENT = 0x00000400;
constexpr long ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT = 0x00000800;
constexpr long ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION = 0x00001000;
constexpr long ODRIVE_ERROR_MOTOR_OVER_TEMP = 0x00002000;
constexpr long ODRIVE_ERROR_INVERTER_OVER_TEMP = 0x00004000;
constexpr long ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION = 0x00008000;
constexpr long ODRIVE_ERROR_POSITION_LIMIT_VIOLATION = 0x00010000;
constexpr long ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED = 0x01000000;
constexpr long ODRIVE_ERROR_ESTOP_REQUESTED = 0x02000000;
constexpr long ODRIVE_ERROR_SPINOUT_DETECTED = 0x04000000;
constexpr long ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED = 0x08000000;
constexpr long ODRIVE_ERROR_THERMISTOR_DISCONNECTED = 0x10000000;
constexpr long ODRIVE_ERROR_CALIBRATION_ERROR = 0x40000000;

}  // namespace ODriveConstants

#endif  // ODRV_API_H