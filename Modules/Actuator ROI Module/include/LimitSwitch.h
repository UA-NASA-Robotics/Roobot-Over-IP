#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stdint.h>

class LimitSwitch {
    private:
        bool _initialized = false;

        const uint8_t _PIN;

    public:
        /**
         * @brief LimitSwitch class constructor
         * 
         * @param pin   The pin to read the limit switch's state from
         */
        LimitSwitch(uint8_t pin = 0xFF);

        /**
         * @brief Initialize the limit switch's Arduino pin
         */
        void init();

        /**
         * @brief Get the state of the limit switch
         * 
         * @return The state of the limit switch
         */
        bool state();
};

#endif