#include "../include/MotorDrivers/IBT2BinaryMotor.h"
#include "../../../../../../lib/ModuleCodec.h"
#include "../include/constants.h"
#include <Arduino.h>

uint8_t IBT2BinaryMotor::maxSpeed() {
    return 1;
}

IBT2BinaryMotor::IBT2BinaryMotor(uint8_t fwd_pin, uint8_t bck_pin) : _FWD_PIN(fwd_pin), _BCK_PIN(bck_pin) {
    _len_control.isBinary(true);
}
        
void IBT2BinaryMotor::init() {
    // Set pins to output
    pinMode(_FWD_PIN, OUTPUT);
    pinMode(_BCK_PIN, OUTPUT);

    // Disable motor
    digitalWrite(_FWD_PIN, 0);
    digitalWrite(_BCK_PIN, 0);
}

void IBT2BinaryMotor::tick(EncoderDriverBase* enc, uint16_t target_length, payloadConstant control_mode) {
    /*
        For a non-binary motor,
        _cur_velocity = enc->velocity();
    */

    // Determine target motor speed
    if (control_mode == ActuatorConstants::LENGTH_MODE) {
        _len_control.setLength(target_length);
        _len_control.tick(enc->value());
        _target_velocity = _len_control.velocity();
    }
    else if (control_mode == ActuatorConstants::VELOCITY_MODE) {
        // Serial.println(_target_velocity);
        // _target_velocity = _target_velocity
    }
    // If control mode was VEL_CONTROL, use the stored value in _target_velocity

    /*
        For a non-binary motor, run PID control here
    */

    // Drive the motors according to the target speed
    if (_target_velocity > 0.0) {
        // Move forwards
        Serial.println("Moving forward");
        digitalWrite(_BCK_PIN, 0);
        digitalWrite(_FWD_PIN, 1);
    }
    else if (_target_velocity < 0.0) {
        // Move backwards
        Serial.println("Moving backwards");
        digitalWrite(_FWD_PIN, 0);
        digitalWrite(_BCK_PIN, 1);
    }
    else {
        digitalWrite(_FWD_PIN, 0);
        digitalWrite(_BCK_PIN, 0);
    }
}