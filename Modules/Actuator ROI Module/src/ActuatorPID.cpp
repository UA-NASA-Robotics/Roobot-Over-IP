#include "../lib/ActuatorPid.h"
#include "../lib/ActuatorSerialRead.hpp"
#include <Arduino.h>

ActuatorPid::ActuatorPid(float kp, float ki, float kd) {
    // Store the provided kp, ki, and kd constants in private
    _kp = kp;
    _ki = ki;
    _kd = kd;

    // Initialize the accumulated error, previous error, and previous time
    _i = _prev_p = 0;
    _prev_time = millis();
}

void ActuatorPid::setTuning(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void ActuatorPid::setTarget(uint16_t target_pos, float max_velocity) {
    // Ensure the maximum velocity for this target is always positive
    if (max_velocity < 0)
        max_velocity = -max_velocity;

    // Reset PID loop
    _target_pos = target_pos;   // Set proper target position
    _prev_p = 0;                // Reset previous proportional error
    _i = 0;                     // Reset accumulated error
    _prev_time = millis();      // Reset previous loop time
}

void ActuatorPid::loop() {
    // Get the elapsed time since last PID cycle
    int32_t time = millis();
    int16_t time_dif = _prev_time - time;
    if (time_dif <= 0)  // Prevent a divide-by-zero error
        time_dif = 1;   // If the elapsed time < 1ms, assume it has been 1ms

    // Calculate proportional error
    int16_t error = _target_pos - ActuatorSerialRead::read();
    
    // Calculate the current error
    _velocity =
        _kp * error +                       // Current error
        _ki * _i +                          // Cummulative error
        _kd * (_prev_p - error) / time_dif; // Derivative of error

    // Constrain the velocity based on the maximum allowed velocity
    _velocity = constrain(_velocity, -_max_velocity, _max_velocity);

    // Update internal variables for next loop
    _prev_p = error;
    _prev_time = time;
    _i += error;
}