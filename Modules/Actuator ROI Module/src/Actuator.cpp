#include "../include/Actuator.h"

bool Actuator::_limitSwitchActivated() {
    switch(_LIMIT_STATE) {
    case NO_LIMITS:
        return false;
    case UPPER_LIMIT:
        return _upper_limit->state();
    case LOWER_LIMIT:
        return _lower_limit->state();
    case BOTH_LIMITS:
        return (_upper_limit->state() && _lower_limit->state());
    }
}

template<class Encoder, class Motor>
Actuator::Actuator(Encoder* enc, Motor* motor, LimitSwitch* upper, LimitSwitch* lower) : _LIMIT_STATE(BOTH_LIMITS) {
    // Initialize flags
    _control_mode = VEL_CONTROL;
    _initialized = false;

    // Create / assign child classes
    _pos_control = PositionController();   // Create a position controller

    // Use provided motor and encoder
    _motor = motor;
    _enc = enc;

    // Use provided limit switches
    _upper_limit = upper;
    _lower_limit = lower;
}

template<class Encoder, class Motor>
Actuator::Actuator(Encoder* enc, Motor* motor, LimitSwitch* limit_switch, uint8_t limit_state) : _LIMIT_STATE(limit_state) {
    // Initialize flags
    _control_mode = VEL_CONTROL;
    _initialized = false;

    // Create / assign child classes
    _pos_control = PositionController();   // Create a position controller

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

template<class Encoder, class Motor>
Actuator::Actuator(Encoder* enc, Motor* motor) : _LIMIT_STATE(NO_LIMITS) {
    // Initialize flags
    _control_mode = VEL_CONTROL;
    _initialized = false;

    // Create / assign child classes
    _pos_control = PositionController();   // Create a position controller

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

    if (_control_mode == VEL_CONTROL) {
        // Velocity control
        _motor->tick();
    }
    else {
        // Position control
        _pos_control.tick();
        _motor->tick();
    }
}

void Actuator::targetVelocity(float vel) {
    _control_mode = VEL_CONTROL;
    _velocity = vel;
}

void Actuator::targetPosition(uint16_t pos) {
    _control_mode = POS_CONTROL;
    _position = pos;
}