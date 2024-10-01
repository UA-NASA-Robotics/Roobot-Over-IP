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
// module chain.

/*----------------- Action Codes -----------------*/
constexpr actionConstant BLANK =
    0b0000000000000000;  // Metadata code for a blank packet (should not be sent)

constexpr actionConstant PING =
    0b0100000000000000;  // Metadata code for a admin Packet that should respond
// if awake and ready, and a module identifier.
constexpr actionConstant PONG =
    0b1100000000000000;  // Metadata code for a admin Packet that should respond
constexpr actionConstant PINGLOOPBACK =
    0b0010000000000000;  // Metadata sent only when a chain message is a PING and the next chain
                         // member is the origin. This is a loopback message so the origin knows the
                         // chain is complete.

constexpr actionConstant STATUSREPORT =
    0b1010000000000000;  // Metadata code for a admin Packet that should.
// elicit status information as a response.

constexpr actionConstant BLACKLIST = 0b0110000000000000;  // Metadata code for a admin Packet that

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
/*--------- Action Codes ----------------*/
constexpr actionConstant SETCONTROLMODE =
    0b0000000000000001;  // Set the control mode of the ODrive (Position, Velocity, etc) atm the
                         // odrive module exclusively supports closed loop control
constexpr actionConstant SETTORQUE =
    0b0000000000000010;  // Set the torque (torque mode) or maximum torque of the ODrive
constexpr actionConstant SETPOSITION =
    0b0000000000000011;  // Set the position(position mode) of the ODrive
constexpr actionConstant SETVELOCITY =
    0b0000000000000100;  // Set the velocity(velocity mode) or maximum velocity of the ODrive
constexpr actionConstant SETMAXACCELERATION =
    0b0000000000000101;  // Set the maximum acceleration of the ODrive
constexpr actionConstant GETCONTROLMDODE =
    0b0000000000000101;  // Get the control mode of the ODrive
constexpr actionConstant GETTORQUESETPOINT =
    0b0000000000000110;  // Get the torque set point of the ODrive
constexpr actionConstant GETPOSITIONSETPOINT =
    0b0000000000000111;  // Get the position set point of the ODrive
constexpr actionConstant GETVELOCITYSETPOINT =
    0b0000000000001000;  // Get the velocity set point of the ODrive
constexpr actionConstant GETMAXACCELERATION =
    0b0000000000001001;  // Get the maximum acceleration of the ODrive

constexpr actionConstant CLEARERRORS =
    0b0000000000001001;  // Clear the errors of the ODrive (can reset the ODrive when in a
                         // non-operational state)
constexpr actionConstant GETERROR = 0b0000000000001010;  // Get the errors of the ODrive

constexpr actionConstant GETPOSITION = 0b0000000000001011;     // Get the position of the ODrive
constexpr actionConstant GETVELOCITY = 0b0000000000001100;     // Get the velocity of the ODrive
constexpr actionConstant GETBUSVOLTAGE = 0b0000000000001101;   // Get the bus voltage of the ODrive
constexpr actionConstant GETCURRENT = 0b0000000000001110;      // Get the current of the ODrive
constexpr actionConstant GETTEMPERATURE = 0b0000000000001111;  // Get the temperature of the ODrive

//------ Set Control Mode Constants ------
constexpr payloadConstant POSITIONMODE = 0b00000000;  // Position mode
constexpr payloadConstant VELOCITYMODE = 0b00000001;  // Velocity mode
constexpr payloadConstant TORQUEMODE = 0b00000010;    // Current mode

//------ Error Codes ------
constexpr payloadConstant NOERROR = 0b00000000;          // No error
constexpr payloadConstant OVERVELOCITY = 0b00000001;     // Over velocity error
constexpr payloadConstant OVERCURRENT = 0b00000010;      // Over current error
constexpr payloadConstant OVERTEMPERATURE = 0b00000011;  // Over temperature error
constexpr payloadConstant UNDERVOLTAGE = 0b00000100;     // Under voltage error
constexpr payloadConstant OVERVOLTAGE = 0b00000101;      // Over voltage error
constexpr payloadConstant MOTORERROR = 0b00000110;       // Motor error
constexpr payloadConstant SENSORLESSERROR = 0b00000111;  // Sensorless error
constexpr payloadConstant ENCDOERERROR = 0b00001000;     // Encoder error
constexpr payloadConstant CONTROLLERERROR = 0b00001001;  // Controller error
constexpr payloadConstant FETTHERMALERROR = 0b00001010;  // FET thermal error

}  // namespace ODriveConstants

#endif