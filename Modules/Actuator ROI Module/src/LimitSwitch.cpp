#include <Arduino.h>
#include <LimitSwitch.h>

LimitSwitch::LimitSwitch(uint8_t pin = 0xFF) : _PIN(pin) {}

void LimitSwitch::init() { pinMode(_PIN, INPUT_PULLUP); }

bool LimitSwitch::state() { return !digitalRead(_PIN); }