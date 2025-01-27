#ifndef ACT_CONT_H
#define ACT_CONT_H

#include "../include/Actuator.h"

class ActuatorContainer {
    private:
        const uint16_t _ENC_LOAD, _ENC_CLK; // Encoder registerload/clock pins
        const uint8_t _SIZE;                // Container size
        uint8_t _count;                     // Number of connected actuators

        Actuator** _actuators;              // Dynamic array of pointer to Actuator

        void _readEncoders();               // Helper function to read the encoder values of all the encoders
        
    public:
        ActuatorContainer(uint8_t size, uint8_t encoder_load, uint8_t encoder_clock);
        ~ActuatorContainer();

        Actuator* operator[](uint8_t index);
        uint8_t getSize();
        uint8_t getCount();

        void connect(Actuator* actuator);
        void init();
        void loop();
};

#endif