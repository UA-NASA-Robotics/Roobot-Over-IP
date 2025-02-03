#ifndef MOTOR_DRIVER_BASE
#define MOTOR_DRIVER_BASE

#include "../Encoders/EncoderDriverBase.h"
#include "LengthController.h"
#include "MotorDriverPid.h"
#include <stdint.h>

class MotorDriverBase {
    protected:
        float _target_velocity;         // Target motor speed (mm/s)
        float _cur_velocity;            // Current motor speed (mm/s)

        LengthController _len_control;  // Length controller
        MotorDriverPid _pid;            // PID controller to control desired speed

    public:
        /**
         * @brief Initialize the Arduino pins for this motor
         */
        virtual void init() = 0;

        /**
         * @brief Update the motor's speed
         */
        virtual void tick(EncoderDriverBase* enc, bool control_mode) = 0;

        /**
         * @brief Initialize the Arduino pins for this motor
         * 
         * @param velocity  The desired speed for the motor in mm/s
         */
        void targetVelocity(float velocity);
};

#endif