#include "../include/Encoders/FirgelliEncoder.h"
#include <Arduino.h>

void FirgelliEncoder::_load() {
    // assert(_initialized)

    // Rising edge the load pin
    digitalWrite(_LOAD, 1);
    delayMicroseconds(250);
    digitalWrite(_LOAD, 0);

    // Flag encoder as loaded
    _loaded = true;
}

void FirgelliEncoder::_read() {
    //assert(_initialized && _loaded)
    
    int32_t time = millis();    // Time of the start of the read
    int16_t pos = 0;            // Position in encoder ticks

    // Shift out current loaded value
    pos = (shiftIn(_SHFT, _CLK, MSBFIRST) << 8);
    pos |= shiftIn(_SHFT, _CLK, MSBFIRST);

    // Update stored encoder values
    _loaded = false;
    _prev_read = _cur_read;
    _cur_read = EncoderReading{pos, time};
}

uint16_t FirgelliEncoder::toMM(EncoderReading reading) {
    return (reading.position / FirgelliEncoder::TICKS_PER_MM);
}
        
uint16_t FirgelliEncoder::toMM(uint16_t rotations) {
    return (rotations / FirgelliEncoder::TICKS_PER_MM);
}

FirgelliEncoder::FirgelliEncoder(uint8_t load, uint8_t clk, uint8_t shft, uint8_t clr) 
: _LOAD(load), _CLK(clk), _SHFT(shft), _CLR(clr) {

}

void FirgelliEncoder::init() {
    // Enable pins
    pinMode(_LOAD, OUTPUT); // Load data to be read
    pinMode(_CLK, OUTPUT);  // Shift a bit of data out
    pinMode(_SHFT, INPUT);  // Data from the shifted bit
    pinMode(_CLR, OUTPUT);  // Clear the counter

    // Clearing the encoder is active low
    digitalWrite(_CLR, 1);
}