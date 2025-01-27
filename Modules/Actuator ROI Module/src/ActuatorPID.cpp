#include <Arduino.h>
#include "../include/ActuatorPid.h"
#include "../include/Encoder.h"

ActuatorPid::ActuatorPid(float kp = 0, float ki = 0, float kd = 0) {
    // Store the provided kp, ki, and kd constants in private
    setTuning(kp, ki, kd);

    // Initialize the accumulated error, previous error, and previous time
    _i = _prev_p = 0;
    _prev_time = millis();

    // Start the PID controller as unpaused
    _paused = false;
}

ActuatorPid::ActuatorPid(PidTuning tuning) {
    // Store copy of provided tuning struct in private
    setTuning(tuning);

    // Initialize the accumulated error, previous error, and previous time
    _i = _prev_p = 0;
    _prev_time = millis();

    // Start the PID controller as unpaused
    _paused = false;
}

void ActuatorPid::setTuning(float kp, float ki, float kd) {
    _tuning.kp = kp;
    _tuning.ki = ki;
    _tuning.kd = kd;
}

void ActuatorPid::setTuning(PidTuning tuning) {
    _tuning = tuning;
}

void ActuatorPid::setTarget(uint16_t target_pos, float max_speed = 1) {
    // Ensure the maximum speed for this target is always positive
    if (max_speed < 0)
        max_speed = -max_speed;

    // Reset PID loop
    _target_pos = target_pos;   // Set proper target position
    _prev_p = 0;                // Reset previous proportional error
    _i = 0;                     // Reset accumulated error
    _prev_time = millis();      // Reset previous loop time
}

void ActuatorPid::loop(EncoderReading reading) {
    // Get the elapsed time since last PID cycle
    int16_t time_dif = _prev_time - reading.time;
    if (time_dif <= 0)  // Prevent a divide-by-zero error
        time_dif = 1;   // If the elapsed time < 1ms, assume it has been 1ms

    // Calculate proportional error
    int16_t error = _target_pos - reading.position;
    
    // Calculate the current target speed
    _speed =
        _tuning.kp * error +                       // Current error
        _tuning.ki * _i +                          // Cummulative error
        _tuning.kd * (_prev_p - error) / time_dif; // Derivative of error

    // Constrain the speed based on the maximum allowed speed
    _speed = _paused ? 0 : constrain(_speed, -_max_speed, _max_speed);

    // Update internal variables for next loop
    _prev_p = error;
    _prev_time = reading.time;
    _i += error;
}

void ActuatorPid::pause() {
    _paused = true;
}

void ActuatorPid::resume() {
    _paused = false;
}

void ActuatorPid::home() {

}