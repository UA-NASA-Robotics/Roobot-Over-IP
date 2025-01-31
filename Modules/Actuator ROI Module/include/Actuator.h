#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "MotorDrivers/MotorDriverBase.h"
#include "Encoders/EncoderDriverBase.h"
#include "PositionController.h"
#include "LimitSwitch.h"

// Control mode states
#define VEL_CONTROL true
#define POS_CONTROL false

// Limit switch states
#define NO_LIMITS 0
#define UPPER_LIMIT 1
#define LOWER_LIMIT 2
#define BOTH_LIMITS 3

class Actuator {
    private:
        bool _initialized;                  // Bool to check if actuator was initialized
        bool _control_mode;                 // Control mode flag (CMF)
        const uint8_t _LIMIT_STATE;         // Limit switch state

        float _velocity;                    // Target velocity (CMF == VEL_CONTROL)
        uint16_t _position;                 // Target position (CMF == POS_CONTROL)

        MotorDriverBase* _motor;            // Motor driver
        EncoderDriverBase* _enc;            // Encoder
        LimitSwitch* _upper_limit;          // Upper limit switch
        LimitSwitch* _lower_limit;          // Lower limit switch

        PositionController _pos_control;    // Position controller

        bool _limitSwitchActivated();       // Returns if a limit switch is active or not

    public:
        template<class Encoder, class Motor>
        Actuator(Encoder* enc, Motor* motor, LimitSwitch* upper, LimitSwitch* lower);

        template<class Encoder, class Motor>
        Actuator(Encoder* enc, Motor* motor, LimitSwitch* limit_switch, uint8_t limit_state);

        template<class Encoder, class Motor>
        Actuator(Encoder* enc, Motor* motor);

        void init();

        void tick();

        void targetVelocity(float vel);

        void targetPosition(uint16_t pos);
};

#endif