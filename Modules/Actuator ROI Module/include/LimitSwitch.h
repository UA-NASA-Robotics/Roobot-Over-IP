#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stdint.h>

class LimitSwitch {
    private:
        bool _initialized = false;

        const uint8_t _PIN;

    public:
        LimitSwitch(uint8_t pin = 0xFF);

        void init();

        bool state();
};

#endif