#ifndef SERREAD_H
#define SERREAD_H

#include <Arduino.h>
#include "../include/ActuatorSerialRead.h"
#include "../include/ActuatorPinout.h"

namespace ActuatorSerialRead {
    uint16_t read() {
        // Use actuator pins namespace in this function block
        using namespace ActuatorPins;

        // Parallel load the value in the 16-bit Up/Down counter
        digitalWrite(PARALLEL_LOAD, 1);
        delayMicroseconds(250);
        digitalWrite(PARALLEL_LOAD, 0);

        // Shift out two byes of data from the 16-bit left shift register
        uint16_t read_value = shiftIn(READ_SERIAL_OUT, SHIFT_CLK, MSBFIRST);
        read_value <<= 8;
        read_value |= shiftIn(READ_SERIAL_OUT, SHIFT_CLK, MSBFIRST);

        // Return encoder value
        return read_value;
    }

    void reset() {
        digitalWrite(ActuatorPins::COUNT_RESET, 1);
        delay(5);
        digitalWrite(ActuatorPins::COUNT_RESET, 0);
    }
}
#endif