#ifndef ACTUATOR_TYPES_H
#define ACTUATOR_TYPES_H

#include <stdint.h>

// Container to store the Arduino pins an actuator is connected to
struct ActuatorPins {
    // Default pins to pin 255
    uint8_t SPEED = 0xFF, DIR = 0xFF, ENC_CLR = 0xFF,
        ENC_SHFT = 0xFF, RET_LMT = 0xFF,  EXT_LMT= 0xFF;
};

// Container to hold the kp, ki, and kd tuning parameters for an actuators PID controller
struct PidTuning {
    float kp = 0;
    float ki = 0;
    float kd = 0;
};

// Container to hold the value of an encoder read
struct EncoderReading {
    int16_t position = 0;
    int32_t time = 0;
};

#endif