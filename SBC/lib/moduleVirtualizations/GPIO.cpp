#include "GPIO.h"

GeneralGPIOModule::GeneralGPIOModule::GeneralGPIOModule(uint8_t moduleOctet,
                                                        TransportAgent& transportAgent)
    : transportAgent(transportAgent) {
    this->moduleOctet = moduleOctet;
    for (int i = 0; i < GeneralGPIOConstants::COUNT;
         i++) {  // Initialize all pins to input mode as default
        pinModes[i] = GeneralGPIOConstants::INPUT_MODE;
        pinValues[i] = 0;
    }
}

GeneralGPIOModule::GeneralGPIOModule::~GeneralGPIOModule() {}

void GeneralGPIOModule::GeneralGPIOModule::ResponseCallback(ROIPackets::Packet packet) {}