#ifndef IBT2_BINARY_MOTOR_H
#define IBT2_BINARY_MOTOR_H

#include "MotorDriverBase.h"
#include <stdint.h>

class IBT2BinaryMotor : protected MotorDriverBase {
    private:
        const uint8_t _FWD_PIN, _BCK_PIN;   // H-bridge pins do drive forward/backward

    public:
        IBT2BinaryMotor(uint8_t fwd_pin, uint8_t bck_pin);
        
        /**
         * @brief Initialize the Arduino pins for this motor
         */
        void init() override;

        /**
         * @brief Update the motor's speed
         */
        void tick() override;
};

#endif