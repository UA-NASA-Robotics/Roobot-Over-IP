#include "../include/ActuatorContainer.h"

void ActuatorContainer::_readEncoders() {
    uint16_t values[_count];
}

ActuatorContainer::ActuatorContainer(uint8_t size, uint8_t encoder_load, uint8_t encoder_clock) :
_ENC_LOAD(encoder_load), _ENC_CLK(encoder_clock), _SIZE(size) {
    _count = 0;

    _actuators = new Actuator*[size];
}

ActuatorContainer::~ActuatorContainer() {
    delete[] _actuators;
}

Actuator* ActuatorContainer::operator[](uint8_t index) {
    return _actuators[index];
}

uint8_t ActuatorContainer::getSize() {
    return _SIZE;
}

uint8_t ActuatorContainer::getCount() {
    return _count;
}

void ActuatorContainer::connect(Actuator* actuator) {
    _actuators[_count++] = actuator;
}

void ActuatorContainer::init() {
    for (int i = 0; i < _count; i++)
        _actuators[i]->loop();
}

void ActuatorContainer::loop() {
    // Load the encoder values into the shift registers
    //Encoder::load(_ENC_CLK, _ENC_LOAD);

    // Run the actuator loops
    for (int i = 0; i < _count; i++)
        _actuators[i]->loop();
}