#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "MotorDrivers/MotorDriverBase.h"
#include "Encoders/EncoderDriverBase.h"
#include "LimitSwitch.h"
#include "constants.h"

class Actuator {
    private:
        bool _initialized;                  // Bool to check if actuator was initialized
        bool _control_mode;                 // Control mode flag (CMF)
        const uint8_t _LIMIT_STATE;         // Limit switch state

        float _velocity;                    // Target velocity (CMF == VEL_CONTROL)
        uint16_t _length;                   // Target length (CMF == LEN_CONTROL)

        MotorDriverBase* _motor;            // Motor driver
        EncoderDriverBase* _enc;            // Encoder
        LimitSwitch* _upper_limit;          // Upper limit switch
        LimitSwitch* _lower_limit;          // Lower limit switch

        bool _limitSwitchActivated();       // Returns if a limit switch is active or not

    public:
        /**
        * @brief Actuator class constructor
        * 
        * @param enc    Pointer to a derived encoder driver for the actuator
        * @param motor  Pointer to a derived motor driver for the actuator
        * @param upper  Pointer to the upper-extension limit switch for the actuator
        * @param lower  Pointer to the lower-extention limit switch for the actuator
        */
        Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, LimitSwitch* upper, LimitSwitch* lower);

        /**
        * @brief Actuator class constructor
        * 
        * @param enc            Pointer to a derived encoder driver for the actuator
        * @param motor          Pointer to a derived motor driver for the actuator
        * @param limit_switch   Pointer to the limit switch for the actuator
        * @param limit_state    The limit switch's location relative to the actuator (UPPER_LIMIT or LOWER_LIMIT)
        */
        Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, LimitSwitch* limit_switch, uint8_t limit_state);

        /**
        * @brief Actuator class constructor
        * 
        * @param enc    Pointer to a derived encoder driver for the actuator
        * @param motor  Pointer to a derived motor driver for the actuator
        */
        Actuator(EncoderDriverBase* enc, MotorDriverBase* motor);

        /**
         * @brief Initialize the actuator and it's components
         */
        void init();

        /**
         * @brief Run a tick on the actuator's motor control loop
         */
        void tick();

        /**
         * @brief Set a target velocity for the actuator's motor
         * 
         * @param vel   The desired target velocity for the actuator's motor in RPM
         */
        void targetVelocity(float vel);

        /**
         * @brief Set a target velocity for the actuator's motor
         * 
         * @param len   The desired target length for the actuator's motor in mm
         */
        void targetLength(uint16_t len);
};

#endif