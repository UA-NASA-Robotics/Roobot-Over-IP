#include "../include/LimitSwitch.h"
#include <Arduino.h>

LimitSwitch::LimitSwitch(uint8_t pin = 0xFF) : _PIN(pin) {}

void LimitSwitch::init() {
    pinMode(_PIN, INPUT);
}

bool LimitSwitch::state() {
    return digitalRead(_PIN);
}