#ifndef supplyVoltage_H
#define supplyVoltage_H

#if defined(__AVR__)
#include <Arduino.h>
#else
// For non-AVR systems
#endif
#include <stdint.h>

#ifndef DEBUG
#define DEBUG false
#endif
namespace supplyVoltageReader {

int getVoltage();  // Read the voltage of the battery the Arduino is currently running on (in
// millivolts) NOTE: You should use getAccurateVCC() instead

uint16_t getAccurateVCC();  // Get the accurate voltage of the battery the Arduino is currently
// running on (in millivolts)

};  // namespace supplyVoltageReader

#endif