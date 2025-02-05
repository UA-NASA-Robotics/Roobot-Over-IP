#ifndef moduleCodec_H
#define moduleCodec_H

#include <stdint.h>

// define if using an Arduino
#if defined(__AVR__)
#include <Arduino.h>
#endif

/*----------------- Constant Types -----------------*/
typedef uint16_t metaConstant;
typedef uint16_t actionConstant;
typedef uint8_t payloadConstant;
typedef uint16_t subDeviceIDConstant;
// Try to use these types where possible to make it easier to understand the purpose of the constant

namespace sysAdminConstants {
// Code and information to be used when building a sysAdminPacket
// Note there is no general packet constants. These should be with individual module sub-classes
// which can assign different meaning to code and data. The sysAdmin network is standardized for all
// modules.

// sysAdmin code bit n is a chain message flag, it should be '|' with any other code that must be
// passed along in the sysAdminPacket. Be careful when requesting all devices on a network to send a
// payload heavy response.

/*----------------- Metadata Codes -----------------*/
constexpr metaConstant NOCHAINMETA =
    0;  // Metadata code for a sysAdminPacket that should not be circulated.
constexpr metaConstant CHAINMESSAGEMETA =
    0b1000000000000000;  // Metadata code for a sysAdminPacket that MUST be circulated around the
// module chain. Note that the reply host address must be |= into the metadata with the chain
// metacode so that replies are sent to the originator of the chain message, not just the previous
// neighbor.

/*----------------- Action Codes -----------------*/
constexpr actionConstant BLANK =
    0b0000000000000000;  // action code for a blank packet (should not be sent)

constexpr actionConstant PING =
    0b0100000000000000;  // action code for a admin Packet that should respond
// if awake and ready, and a module identifier.
constexpr actionConstant PONG =
    0b1100000000000000;  // action code for a admin Packet that should respond
constexpr actionConstant PINGLOOPBACK =
    0b0010000000000000;  // action sent only when a chain message is a PING and the next chain
                         // member is the origin. This is a loopback message so the origin knows the
                         // chain is complete.

constexpr actionConstant STATUSREPORT =
    0b1010000000000000;  // action code for a admin Packet that should.
// elicit status information as a response.

constexpr actionConstant BLACKLIST = 0b0110000000000000;  // action code for a admin Packet that

}  // namespace sysAdminConstants

namespace moduleTypesConstants {

typedef uint16_t moduleTypeConstant;

/*--------- Module ID Codes ----------------*/
// We are skipping 0 and 1 as they may be used for fail or error codes
constexpr moduleTypeConstant MasterSBC =
    2;  // The MasterSBC module returns a 2 as it's id in a ping
constexpr moduleTypeConstant GeneralGPIO =
    3;                                    // A generalGPIO module returns a 3 as it's id in a ping
constexpr moduleTypeConstant ODrive = 4;  // An ODrive module returns a 4 as it's id in a ping
constexpr moduleTypeConstant Actuator = 5;
}  // namespace moduleTypesConstants

namespace statusReportConstants {

typedef uint8_t statusConstant;

constexpr statusConstant NULLCODE = 0;             // No status code/invalid status code
constexpr statusConstant OPERATING = 1;            // Operating normally, no errors
constexpr statusConstant OPERATINGWITHERRORS = 2;  // Operating with soft errors
constexpr statusConstant OPERATINGWITHOUTCHAIN =
    3;                                      // Operating normally, but unable to form network chain
constexpr statusConstant NOTOPERABLE = 4;   // Not operable, hard error
constexpr statusConstant INITIALIZING = 5;  // Initializing, not ready for operation
constexpr statusConstant BLANKSTATE = 6;    // Blank state, Device is ready to operate, but requires
// configuration before use. Use to signal a device that
// has been freshly powered on or reset.

}  // namespace statusReportConstants

namespace blacklistConstants {
typedef uint8_t blacklistConstant;

constexpr blacklistConstant NULLCODE = 0;     // No blacklist code/invalid blacklist code
constexpr blacklistConstant BLACKLISTED = 1;  // Blacklisted, do not send messages to this device

constexpr blacklistConstant ADDBLACKLIST = 2;     // Add this device to the blacklist
constexpr blacklistConstant REMOVEBLACKLIST = 3;  // Remove this device from the blacklist
constexpr blacklistConstant LISTBLACKLIST = 4;    // Clear the blacklist

}  // namespace blacklistConstants
namespace GeneralGPIOConstants {
/*--------- Action Codes ----------------*/
constexpr actionConstant SET_PIN_MODE = 0b0000000000000001;  // Set the mode of a pin
constexpr actionConstant SET_OUTPUT = 0b0000000000000010;    // Set the output of a pin
constexpr actionConstant READ = 0b0000000000000011;  // Read the value of a pin, digital or analog

/*--------- Subdevice ID Codes ----------------*/
constexpr subDeviceIDConstant DIGITAL_PIN_0 = 0;  // Pin 0
constexpr subDeviceIDConstant DIGITAL_PIN_1 = 1;  // Pin 1
constexpr subDeviceIDConstant DIGITAL_PIN_2 = 2;  // Pin 2
constexpr subDeviceIDConstant DIGITAL_PIN_3 = 3;  // Pin 3
constexpr subDeviceIDConstant DIGITAL_PIN_4 = 4;  // Pin 4
constexpr subDeviceIDConstant DIGITAL_PIN_5 = 5;  // Pin 5
constexpr subDeviceIDConstant DIGITAL_PIN_6 = 6;  // Pin 6
constexpr subDeviceIDConstant DIGITAL_PIN_7 = 7;  // Pin 7

// Pins 8-13 are reserved for the SPI interface and other functions

constexpr subDeviceIDConstant ANALOG_PIN_0 = 10;  // Pin 0
constexpr subDeviceIDConstant ANALOG_PIN_1 = 11;  // Pin 1
constexpr subDeviceIDConstant ANALOG_PIN_2 = 12;  // Pin 2
constexpr subDeviceIDConstant ANALOG_PIN_3 = 13;  // Pin 3
constexpr subDeviceIDConstant ANALOG_PIN_4 = 14;  // Pin 4
constexpr subDeviceIDConstant ANALOG_PIN_5 = 15;  // Pin 5
constexpr subDeviceIDConstant ANALOG_PIN_6 = 16;  // Pin 6 - Analog read only
constexpr subDeviceIDConstant ANALOG_PIN_7 = 17;  // Pin 7 - Analog read only

constexpr subDeviceIDConstant COUNT = 18;  // Number of sub devices

#if defined(__AVR__)
constexpr uint8_t subDeviceIDLookup[] = {
    0, 1,  2,  3,  4,  5,  6,  7, 0,
    0, A1, A2, A3, A4, A5, A6, A7};  // Index is subdevice ID, value is pin number
#endif

/*--------- Payload Codes ----------------*/

constexpr payloadConstant INPUT_MODE = 0b00000001;         // Set the pin mode to input
constexpr payloadConstant INPUT_PULLUP_MODE = 0b00000010;  // Set the pin mode to input pullup
constexpr payloadConstant OUTPUT_MODE = 0b00000011;        // Set the pin mode to output

}  // namespace GeneralGPIOConstants

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
constexpr actionConstant SETCONTROLMODE =
    MaskConstants::SETMASK &
    MaskConstants::ControlMode;  // Set the control mode of the ODrive (Position, Velocity, etc) atm
                                 // the odrive module exclusively supports closed loop control
constexpr actionConstant SETINPUTMODE =
    MaskConstants::SETMASK &
    MaskConstants::InputMode;  // Set the input mode of the ODrive (Position, Velocity, etc)
constexpr actionConstant SETTORQUE =
    MaskConstants::SETMASK &
    MaskConstants::Torque;  // Set the torque (torque mode) or maximum torque of the ODrive
constexpr actionConstant SETPOSITION =
    MaskConstants::SETMASK &
    MaskConstants::PositionSetPoint;  // Set the position(position mode) of the ODrive
constexpr actionConstant SETRELATIVEPOSITION =
    MaskConstants::SETMASK &
    MaskConstants::PositionRelative;  // Set the relative position(position mode) of the ODrive
constexpr actionConstant SETVELOCITY =
    MaskConstants::SETMASK & MaskConstants::VelocitySetPoint;  // Set the velocity(velocity mode) or
                                                               // maximum velocity of the ODrive

constexpr actionConstant GETCONTROLMDODE =
    MaskConstants::GETMASK & MaskConstants::ControlMode;  // Get the control mode of the ODrive
constexpr actionConstant GETINPUTMODE =
    MaskConstants::GETMASK & MaskConstants::InputMode;  // Get the input mode of the ODrive
constexpr actionConstant GETTORQUESETPOINT =
    MaskConstants::GETMASK & MaskConstants::Torque;  // Get the torque set point of the ODrive
constexpr actionConstant GETPOSITIONSETPOINT =
    MaskConstants::GETMASK &
    MaskConstants::PositionSetPoint;  // Get the position set point of the ODrive
constexpr actionConstant GETVELOCITYSETPOINT =
    MaskConstants::GETMASK &
    MaskConstants::VelocitySetPoint;  // Get the velocity set point of the ODrive

constexpr actionConstant CLEARERRORS =
    MaskConstants::SETMASK & MaskConstants::Error;  // Clear the errors of the ODrive (can reset the
                                                    // ODrive when in a non-operational state)
constexpr actionConstant GETERROR =
    MaskConstants::GETMASK & MaskConstants::Error;  // Get the errors of the ODrive

constexpr actionConstant GETPOSITION =
    MaskConstants::GETMASK & MaskConstants::Position;  // Get the position of the ODrive
constexpr actionConstant GETVELOCITY =
    MaskConstants::GETMASK & MaskConstants::Velocity;  // Get the velocity of the ODrive
constexpr actionConstant GETBUSVOLTAGE =
    MaskConstants::GETMASK & MaskConstants::BusVoltage;  // Get the bus voltage of the ODrive
constexpr actionConstant GETCURRENT =
    MaskConstants::GETMASK & MaskConstants::Current;  // Get the current of the ODrive
constexpr actionConstant GETFETTEMPERATURE =
    MaskConstants::GETMASK &
    MaskConstants::FETTemperature;  // Get the temperature of the ODrive transistors
constexpr actionConstant GETMOTORTEMPERATURE =
    MaskConstants::GETMASK &
    MaskConstants::MotorTemperature;  // Get the temperature of the ODrive motor

constexpr actionConstant GETKINEMATICFEEDBACK =
    MaskConstants::GETMASK &
    MaskConstants::KinematicFeedback;  // Get the kinematic feedback of the ODrive

constexpr actionConstant GETALL =
    MaskConstants::GETMASK & MaskConstants::all;  // Get all values of the ODrive

//------ Set Control Mode Constants ------
constexpr payloadConstant POSITIONMODE = 0b00000000;  // Position mode
constexpr payloadConstant VELOCITYMODE = 0b00000001;  // Velocity mode
constexpr payloadConstant TORQUEMODE = 0b00000010;    // Current mode

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

namespace ActuatorConstants {
namespace MaskConstants {
constexpr uint16_t SET_MASK = 0x0000;   // Set mask
constexpr uint16_t GET_MASK = 0x8000;   // Get mask
constexpr uint8_t LENGTH = 0x00;        // Length operations
constexpr uint8_t VELOCITY = 0x01;      // Velocity operations
constexpr uint8_t HOMING = 0x02;        // Homing operations
constexpr uint8_t CONTROL_FLOW = 0x03;  // Control flow operations
constexpr uint8_t STATE_FLOW = 0x04;    // State flow operations

} // namespace MaskContstants

/*--------- Action Codes ----------------*/
constexpr actionConstant SET_RELATIVE_LENGTH =
    MaskConstants::SET_MASK &
    MaskConstants::LENGTH & 0x0;

constexpr actionConstant SET_ABSOLUTE_LENGTH = 
    MaskConstants::SET_MASK &
    MaskConstants::LENGTH & 0x1;

constexpr actionConstant GET_TARGET_LENGTH = 
    MaskConstants::GET_MASK &
    MaskConstants::LENGTH & 0x2;
    
constexpr actionConstant GET_CURRENT_LENGTH = 
    MaskConstants::GET_MASK &
    MaskConstants::LENGTH & 0x3;

constexpr actionConstant SET_VELOCITY = 
    MaskConstants::SET_MASK &
    MaskConstants::VELOCITY & 0x0;

constexpr actionConstant GET_TARGET_VELOCITY = 
    MaskConstants::GET_MASK &
    MaskConstants::VELOCITY & 0x1;

constexpr actionConstant GET_CURRENT_VELOCITY = 
    MaskConstants::GET_MASK &
    MaskConstants::VELOCITY & 0x2;

constexpr actionConstant SET_HOME_MAX = 
    MaskConstants::SET_MASK &
    MaskConstants::HOMING & 0x0;

constexpr actionConstant SET_HOME_MIN = 
    MaskConstants::SET_MASK &
    MaskConstants::HOMING & 0x1;

constexpr actionConstant GET_HOMED = 
    MaskConstants::GET_MASK &
    MaskConstants::HOMING & 0x2;

constexpr actionConstant SET_CONTROL = 
    MaskConstants::SET_MASK &
    MaskConstants::CONTROL_FLOW & 0x0;

constexpr actionConstant GET_CONTROL = 
    MaskConstants::GET_MASK &
    MaskConstants::CONTROL_FLOW & 0x1;

constexpr actionConstant SET_SPEED_PID = 
    MaskConstants::SET_MASK &
    MaskConstants::STATE_FLOW & 0x0;

constexpr actionConstant SET_LENGTH_PID = 
    MaskConstants::SET_MASK &
    MaskConstants::STATE_FLOW & 0x1;

constexpr actionConstant GET_SPEED_PID = 
    MaskConstants::GET_MASK &
    MaskConstants::STATE_FLOW & 0x2;

constexpr actionConstant GET_LENGTH_PID = 
    MaskConstants::GET_MASK &
    MaskConstants::STATE_FLOW & 0x3;

//------ Set Control Mode Constants ------
constexpr payloadConstant POSITION_MODE = 0b00000000;  // Position mode
constexpr payloadConstant VELOCITY_MODE = 0b00000001;  // Velocity mode

} // namespace ActuatorConstants

namespace WatchdogConstants {
constexpr uint16_t MAINTAIN_SLEEP_TIME = 50;  // The time to sleep between maintainState
                                              // loops, in milliseconds for the ROS nodes.

constexpr uint16_t WATCHDOG_TIMEOUT =
    MAINTAIN_SLEEP_TIME * 5;  // The time in ms before the watchdog times out
}  // namespace WatchdogConstants

#endif