#include "../include/EncoderDrivers/PLoadRShiftEncoder.h"
#include <Arduino.h>

void PLoadRShiftEncoder::_load() {
    // assert(_initialized)

    // Rising edge the load pin
    digitalWrite(_LOAD, 1);
    delayMicroseconds(250);
    digitalWrite(_LOAD, 0);

    // Flag encoder as loaded
    _loaded = true;
}

void PLoadRShiftEncoder::_read() {
    //assert(_initialized && _loaded)
    
    int32_t cur_time = millis();    // Time of the start of the read
    int16_t len = 0;            // Length in encoder ticks

    // Shift out current loaded value
    len = (shiftIn(_SHFT, _CLK, MSBFIRST) << 8);
    len |= shiftIn(_SHFT, _CLK, MSBFIRST);

    // Update stored encoder values
    _loaded = false;
    _prev_read.length = _cur_read.length;
    _prev_read.time = _cur_read.time;
    _cur_read.length = len / TICKS_PER_MM + _homed_length;
    _cur_read.time = cur_time;
}

PLoadRShiftEncoder::PLoadRShiftEncoder(uint8_t load, uint8_t clk, uint8_t shft, uint8_t clr) 
: _LOAD(load), _CLK(clk), _SHFT(shft), _CLR(clr) {
    _homed_length = 0;
}

void PLoadRShiftEncoder::clear() {
    // Reset the counter
    digitalWrite(_CLR, 0);
    delayMicroseconds(250);
    digitalWrite(_CLR, 1);

    // Reset encoder reads
    _cur_read.length = _prev_read.length = 0;
    _cur_read.time = _cur_read.time = millis();
}

void PLoadRShiftEncoder::init() {
    // Enable pins
    pinMode(_SHFT, INPUT);  // Data from the shifted bit
    pinMode(_LOAD, OUTPUT); // Load data to be read
    pinMode(_CLK, OUTPUT);  // Shift a bit of data out
    pinMode(_CLR, OUTPUT);  // Clear the counter

    // Clear the external counter
    clear();
}

void PLoadRShiftEncoder::home(uint16_t length) {
    // Run original functionality
    EncoderDriverBase::home(length);

    // Clear the external counter
    clear();
}