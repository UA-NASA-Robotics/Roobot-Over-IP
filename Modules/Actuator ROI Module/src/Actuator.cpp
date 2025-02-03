#include "../include/Actuator.h"

bool Actuator::_limitSwitchActivated() {
    switch(_LIMIT_STATE) {
    case UPPER_LIMIT:
        return _upper_limit->state();
    case LOWER_LIMIT:
        return _lower_limit->state();
    case BOTH_LIMITS:
        return (_upper_limit->state() && _lower_limit->state());
    default:
        return false;
    }
}

Actuator::Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, LimitSwitch* upper, LimitSwitch* lower) : _LIMIT_STATE(BOTH_LIMITS) {
    // Initialize flags
    _control_mode = VEL_CONTROL;
    _initialized = false;

    // Use provided motor and encoder
    _motor = motor;
    _enc = enc;

    // Use provided limit switches
    _upper_limit = upper;
    _lower_limit = lower;
}

Actuator::Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, LimitSwitch* limit_switch, uint8_t limit_state) : _LIMIT_STATE(limit_state) {
    // Initialize flags
    _control_mode = VEL_CONTROL;
    _initialized = false;

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

Actuator::Actuator(EncoderDriverBase* enc, MotorDriverBase* motor) : _LIMIT_STATE(NO_LIMITS) {
    // Initialize flags
    _control_mode = VEL_CONTROL;
    _initialized = false;

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

    // Tick the motor
    if (_limitSwitchActivated())
        targetVelocity(0);
    _motor->tick(_enc, _control_mode);
        
}

void Actuator::targetVelocity(float vel) {
    _control_mode = VEL_CONTROL;
    _velocity = vel;
}

void Actuator::targetLength(uint16_t len) {
    _control_mode = LEN_CONTROL;
    _length = len;
}