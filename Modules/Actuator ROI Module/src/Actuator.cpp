#include "../../../../../../lib/ModuleCodec.h"
#include "../include/Actuator.h"
#include "constants.h"
#include <Arduino.h>

bool Actuator::_limitSwitchActivated(uint8_t state = 0xFF) {
    state = (state == 0xFF ? _LIMIT_STATE : state);
    
    switch(state) {
        case UPPER_LIMIT:
        return _upper_limit->state() && _velocity > 0;

        case LOWER_LIMIT:
        return _lower_limit->state() && _velocity < 0;

        case BOTH_LIMITS:
        return (_limitSwitchActivated(UPPER_LIMIT) || _limitSwitchActivated(LOWER_LIMIT));

        default:
        return false;
    }
}

void Actuator::_passiveHome() {
    _homed_time = millis();
    _enc->home(_MIN_LENGTH);

    Serial.println("Passive home");
}

Actuator::Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, LimitSwitch* upper, LimitSwitch* lower, uint16_t min_length, uint16_t max_length)
: _LIMIT_STATE(BOTH_LIMITS), _MAX_LENGTH(max_length), _MIN_LENGTH(min_length) {
    // Use provided motor and encoder
    _motor = motor;
    _enc = enc;

    // Use provided limit switches
    _upper_limit = upper;
    _lower_limit = lower;
}

Actuator::Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, LimitSwitch* limit_switch, uint8_t limit_state, uint16_t min_length, uint16_t max_length)
: _LIMIT_STATE(BOTH_LIMITS), _MAX_LENGTH(max_length), _MIN_LENGTH(min_length) {
    // Use provided motor and encoder
    _motor = motor;
    _enc = enc;

    // Use provided limit switch
    if (_LIMIT_STATE == UPPER_LIMIT) {
        _upper_limit = limit_switch;
        _lower_limit = nullptr;
    }
    else {
        _upper_limit = nullptr;
        _lower_limit = limit_switch;
    }
}

Actuator::Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, uint16_t min_length, uint16_t max_length)
: _LIMIT_STATE(BOTH_LIMITS), _MAX_LENGTH(max_length), _MIN_LENGTH(min_length) {
    // Use provided motor and encoder
    _motor = motor;
    _enc = enc;

    // Use provided limit switches or a dummy limit switch
    _upper_limit = nullptr;
    _lower_limit = nullptr;
}

void Actuator::init() {
    // Init motor and encoder
    _motor->init();
    _enc->init();

    // Init limit switches
    if (_upper_limit) _upper_limit->init();
    if (_lower_limit) _lower_limit->init();

    // Flag actuator as initialized
    _initialized = true;
}

void Actuator::tick() {
    // Ensure the actuator has been initialized
    //assert(_initialized);

    // Tick the encoder
    _enc->tick();

    // Check if there is a lower limit switch to passively home to
    if (_LIMIT_STATE == LOWER_LIMIT || _LIMIT_STATE == UPPER_LIMIT) {
        // Check if the actuator is at min extention to passive home
        if (_limitSwitchActivated(LOWER_LIMIT) && _enc->velocity() == 0)
            _passiveHome();
    }

    // Trying to move down but the encoder is reading zero
    else if (_enc->velocity() == 0 && _velocity < 0) {
        _passiveHome();
    }
        
    // Check if any limit switch is activated to stop moving
    if (_limitSwitchActivated()) //|| _enc->velocity() == 0)
        setVelocity(0);

    // Tick the motor
    _motor->setVelocity(_velocity);
    _motor->tick(_enc, _length, _control_mode);       
}

void Actuator::setVelocity(float vel) {
    _velocity = vel;
}

void Actuator::setRelativeLength(uint16_t rel_len) {
    _length += rel_len;
}

void Actuator::setAbsoluteLength(uint16_t abs_len) {
    _length = abs_len;
}

void Actuator::setControlMode(payloadConstant control_mode) {
    _control_mode = control_mode;
}