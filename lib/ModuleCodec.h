#ifndef moduleCodec_H
#define moduleCodec_H

#include <stdint.h>

namespace sysAdminConstants {
// Code and information to be used when building a sysAdminPacket
// Note there is no general packet constants. These should be with individual module sub-classes
// which can assign different meaning to code and data. The sysAdmin network is standardized for all
// modules.

// sysAdmin code bit n is a chain message flag, it should be '|' with any other code that must be
// passed along in the sysAdminPacket. Be careful when requesting all devices on a network to send a
// payload heavy response.

const uint16_t NOCHAINMETA =
    0;  // Metadata code for a sysAdminPacket that should not be circulated.
const uint16_t CHAINMESSAGEMETA =
    0b1000000000000000;  // Metadata code for a sysAdminPacket that MUST be circulated around the
                         // module chain.

/*----------------- Action Codes -----------------*/

const uint16_t PING = 0b0100000000000000;  // Metadata code for a admin Packet that should respond
                                           // if awake and ready, and a module identifier.
const uint16_t PONG = 0b0010000000000000;  // Metadata code for a admin Packet that should respond

const uint16_t STATUSREPORT = 0b0010000000000000;  // Metadata code for a admin Packet that should
                                                   // elicit status information as a response.

}  // namespace sysAdminConstants

namespace moduleTypesConstants {
/*--------- Module ID Codes ----------------*/
// We are skipping 0 and 1 as they may be used for fail or error codes
const uint16_t MasterSBC = 2;    // The MasterSBC module returns a 2 as it's id in a ping
const uint16_t GeneralGPIO = 3;  // A generalGPIO module returns a 3 as it's id in a ping
}  // namespace moduleTypesConstants

namespace statusReportConstants {

const uint8_t NULLCODE = 0;               // No status code/invalid status code
const uint8_t OPERATING = 1;              // Operating normally, no errors
const uint8_t OPERATINGWITHERRORS = 2;    // Operating with soft errors
const uint8_t OPERATINGWITHOUTCHAIN = 3;  // Operating normally, but unable to form network chain
const uint8_t NOTOPERABLE = 4;            // Not operable, hard error
const uint8_t INITIALIZING = 5;           // Initializing, not ready for operation
const uint8_t BLANKSTATE = 6;             // Blank state, Device is ready to operate, but requires
                                          // configuration before use. Use to signal a device that
                                          // has been freshly powered on or reset.

}  // namespace statusReportConstants
namespace GeneralGPIOConstants {
/*--------- Action Codes ----------------*/
const uint16_t SET_PIN_MODE = 0b0000000000000001;  // Set the mode of a pin
const uint16_t SET_OUTPUT = 0b0000000000000010;    // Set the output of a pin
const uint16_t READ = 0b0000000000000011;          // Read the value of a pin, digital or analog

/*--------- Subdevice ID Codes ----------------*/
const uint16_t DIGITAL_PIN_0 = 0;  // Pin 0
const uint16_t DIGITAL_PIN_1 = 1;  // Pin 1
const uint16_t DIGITAL_PIN_2 = 2;  // Pin 2
const uint16_t DIGITAL_PIN_3 = 3;  // Pin 3
const uint16_t DIGITAL_PIN_4 = 4;  // Pin 4
const uint16_t DIGITAL_PIN_5 = 5;  // Pin 5
const uint16_t DIGITAL_PIN_6 = 6;  // Pin 6
const uint16_t DIGITAL_PIN_7 = 7;  // Pin 7

const uint16_t ANALOG_PIN_0 = 10;  // Pin 0
const uint16_t ANALOG_PIN_1 = 11;  // Pin 1
const uint16_t ANALOG_PIN_2 = 12;  // Pin 2
const uint16_t ANALOG_PIN_3 = 13;  // Pin 3
const uint16_t ANALOG_PIN_4 = 14;  // Pin 4
const uint16_t ANALOG_PIN_5 = 15;  // Pin 5
const uint16_t ANALOG_PIN_6 = 16;  // Pin 6 - Analog read only
const uint16_t ANALOG_PIN_7 = 17;  // Pin 7 - Analog read only

const uint8_t subDeviceIDLookup[] = {
    2, 3,  4,  5,  6,  7,  8,  9, 0,
    0, A1, A2, A3, A4, A5, A6, A7};  // Index is subdevice ID, value is pin number

/*--------- Payload Codes ----------------*/

const uint16_t INPUT_MODE = 0b0000000000000001;         // Set the pin mode to input
const uint16_t INPUT_PULLUP_MODE = 0b0000000000000010;  // Set the pin mode to input pullup
const uint16_t OUTPUT_MODE = 0b0000000000000011;        // Set the pin mode to output

}  // namespace GeneralGPIOConstants

#endif