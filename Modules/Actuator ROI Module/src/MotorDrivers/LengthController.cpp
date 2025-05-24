#include "../../include/MotorDrivers/LengthController.h"

LengthController::LengthController() { _is_binary = true; }

void LengthController::isBinary(bool is_binary) { _is_binary = is_binary; }

void LengthController::setLength(int16_t target_len) { _target_len = target_len; }

float LengthController::velocity() { return _target_vel; }

void LengthController::tick(EncoderReading reading) {
    if (_is_binary) {
        // Target velocity becomes a number with a magnitude of the desired speed
        _target_vel = _target_len - reading.length;
    } else {
        // To be implemented: PID control
    }
}