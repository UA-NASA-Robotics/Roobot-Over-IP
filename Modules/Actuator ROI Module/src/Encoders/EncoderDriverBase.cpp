#include "../../include/Encoders/EncoderDriverBase.h"

void EncoderDriverBase::tick() {
    _load();
    _read();
}

void EncoderDriverBase::home(uint16_t length) {
    _homed_length = length;
}

EncoderReading EncoderDriverBase::value() {
    return _cur_read;
}

float EncoderDriverBase::velocity() {
    // Calculate the time difference between the last reads to ensure it isn't 0 or less
    int32_t time_dif = _cur_read.time - _prev_read.time;

    // If time has elapsed, return the encoder's average speed in mm/s
    if (time_dif > 0)
        return 1000 * (_cur_read.length - _prev_read.length) / time_dif;
        
    // Otherwise, return 0
    return 0;
}
