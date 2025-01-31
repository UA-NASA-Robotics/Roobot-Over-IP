#ifndef MOTOR_DRIVER_BASE
#define MOTOR_DRIVER_BASE

#include "MotorDriverPid.h"
#include <stdint.h>

class MotorDriverBase {
    protected:
        float _target_velocity; // Target motor speed (rad/s)
        float _cur_velocity;    // Current motor speed (rad/s)
        MotorDriverPid _pid;    // PID controller to control desired speed

    public:
        /**
         * @brief Initialize the Arduino pins for this motor
         */
        virtual void init() = 0;

        /**
         * @brief Update the motor's speed
         */
        virtual void tick() = 0;

        /**
         * @brief Initialize the Arduino pins for this motor
         * 
         * @param velocity  The desired speed for the motor in rad/s
         */
        virtual void targetVelocity(float velocity) = 0;
};

#endif