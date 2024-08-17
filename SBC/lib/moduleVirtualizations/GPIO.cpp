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

void GeneralGPIOModule::GeneralGPIOModule::ResponseCallback(ROIPackets::Packet packet) {
    // This function is called when a response packet is received
    switch (packet.getActionCode()) {
        case GeneralGPIOConstants::SET_OUTPUT:
#ifdef DEBUG
            std::cout << "GPIO Module: SET_OUTPUT response received" << std::endl;
#endif
            // state is set on send, so no need to do anything here
            break;
        case GeneralGPIOConstants::SET_PIN_MODE:
#ifdef DEBUG
            std::cout << "GPIO Module: SET_PIN_MODE response received" << std::endl;
#endif
            // state is set on send, so no need to do anything here
            break;
        case GeneralGPIOConstants::READ:
#ifdef DEBUG
            std::cout << "GPIO Module: READ response received" << std::endl;
#endif
            data[ROIConstants::ROIMAXPACKETPAYLOAD];
            packet.getData(data, ROIConstants::ROIMAXPACKETPAYLOAD);
            pinValues[packet.getSubDeviceID()] =
                packet.getSubDeviceID() >= 10
                    ? data[1] << 8 | data[0]
                    : data[0];  // If the pin is a 16 bit pin, then the data is stored in big endian
                                // format, else it is one byte
    }
}

void GeneralGPIOModule::GeneralGPIOModule::MaintainState() {
    // This function is called periodically to maintain the state of the GPIO pins
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        if (pinModes[i] == GeneralGPIOConstants::INPUT_MODE ||
            pinModes[i] == GeneralGPIOConstants::INPUT_PULLUP_MODE) {
            // If the pin is in output mode, then push the state to the module
        }
    }
}