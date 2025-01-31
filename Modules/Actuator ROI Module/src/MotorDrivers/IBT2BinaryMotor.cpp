#include "../include/MotorDrivers/IBT2BinaryMotor.h"
#include <Arduino.h>

IBT2BinaryMotor::IBT2BinaryMotor(uint8_t fwd_pin, uint8_t bck_pin) : _FWD_PIN(fwd_pin), _BCK_PIN(bck_pin) {

}
        
void IBT2BinaryMotor::init() {
    // Set pins to output
    pinMode(_FWD_PIN, OUTPUT);
    pinMode(_BCK_PIN, OUTPUT);
}

void IBT2BinaryMotor::tick() {
    if (_target_velocity >= 1) {
        portWrite(_FWD_PIN, 1);
        portWrite(_BCK_PIN, 0);
    }
    else if (_target_velocity <= -1) {
        portWrite(_FWD_PIN, 0);
        portWrite(_BCK_PIN, 1);
    }
    else {
        portWrite(_FWD_PIN, 0);
        portWrite(_BCK_PIN, 0);
    }
}

void IBT2BinaryMotor::targetVelocity(float velocity) {
    _target_velocity = velocity;
}