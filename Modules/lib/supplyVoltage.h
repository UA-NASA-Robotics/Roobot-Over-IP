#ifndef supplyVoltage_H
#define supplyVoltage_H

#include <Arduino.h>
#include <stdint.h>

namespace supplyVoltageReader {

    int readVcc();  // Read the voltage of the battery the Arduino is currently running on (in
    // millivolts) NOTE: You should use getAccurateVCC() instead

    uint16_t getAccurateVCC();  // Get the accurate voltage of the battery the Arduino is currently
    // running on (in millivolts)

};  // namespace supplyVoltageReader

#endif