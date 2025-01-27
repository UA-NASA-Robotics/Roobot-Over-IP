#include <Arduino.h>
#include "../include/Actuator.h"

Actuator::Actuator(ActuatorPins pins = {}, uint16_t travel_limit = 0, PidTuning tuning) :
_PINS(pins), _TRAVEL_LIMIT(travel_limit) {
    //_enc = Encoder(_PINS.ENC_CLR, _PINS.ENC_SHFT);
}

void Actuator::init() {
    // Initialize the pins on the Arduino
    pinMode(_PINS.SPEED, OUTPUT);
    pinMode(_PINS.DIR, OUTPUT);
    pinMode(_PINS.ENC_CLR, OUTPUT);
    pinMode(_PINS.ENC_SHFT, OUTPUT);
    pinMode(_PINS.RET_LMT, INPUT);
    pinMode(_PINS.EXT_LMT, INPUT);
}

void Actuator::loop() {
    
}