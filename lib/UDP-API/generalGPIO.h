#ifndef GENERAL_GPIO_H
#define GENERAL_GPIO_H

#include "packetTypes.h"

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

#endif  // GENERAL_GPIO_H