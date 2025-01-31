#include "../../include/Encoders/EncoderDriverBase.h"

EncoderReading EncoderDriverBase::value() {
    return _cur_read;
}

float EncoderDriverBase::velocity() {
    // Calculate the time difference between the last reads to ensure it isn't 0 or less
    int32_t time_dif = _cur_read.time - _prev_read.time;

    // If time has elapsed, return the encoder's average speed in RPM
    // Time is in milliseconds, thus the result must be multiplied by 60,000 ms/min
    if (time_dif > 0)
        return 60000 * (_cur_read.position - _prev_read.position) / time_dif;
    return 0;
}

void EncoderDriverBase::tick() {
    _load();
    _read();
}