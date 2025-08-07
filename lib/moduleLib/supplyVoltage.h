#ifndef supplyVoltage_H
#define supplyVoltage_H

#if defined(__AVR__)
#include <Arduino.h>
#else
// For non-AVR systems
#endif
#include <stdint.h>

namespace supplyVoltageReader {

/**
 * @brief Read the supply voltage of the Arduino, NOTE: INTERNAL USE ONLY, use getAccurateVCC()
 *
 * @return int, the supply voltage of the Arduino in millivolts
 */
int getVoltage();

/**
 * @brief Get the an accurate reading of the supply voltage of the Arduino
 *
 * @return uint16_t, the supply voltage of the Arduino in millivolts
 */
uint16_t getAccurateVCC();

};  // namespace supplyVoltageReader

#endif