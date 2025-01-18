#ifndef SERREAD_H
#define SERREAD_H

#include <stdint.h>

namespace ActuatorSerialRead {
    /*
     * @brief Read the 16-bit encoder value from the actuator hardware's 16-bit counter
     *
     * @return The current actuator's encoder value
     */
    uint16_t read();

    /*
     * @brief Reset the 16-bit encoder value from the actuator hardware's 16-bit counter
     *
     */
    void reset();
}

#endif