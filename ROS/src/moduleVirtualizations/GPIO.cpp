#include "GPIO.h"

/*----- Private Functions -----*/

void GeneralGPIOModule::responseCallback(ROIPackets::Packet packet) {
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
            uint8_t data[ROIConstants::ROIMAXPACKETPAYLOAD];
            packet.getData(data, ROIConstants::ROIMAXPACKETPAYLOAD);
            pinValues[packet.getSubDeviceID()] =
                packet.getSubDeviceID() >= 10
                    ? data[1] << 8 | data[0]
                    : data[0];  // If the pin is a 16 bit pin, then the data is stored in big endian
                                // format, else it is one byte
    }
}

void GeneralGPIOModule::maintainState() {
    // This function is called periodically to maintain the state of the GPIO pins
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        if (i > 7 && i < 10) {
            i = 9;  // skip empty slots... next real slot is 10
            continue;
        }
        if (pinModes[i] == GeneralGPIOConstants::INPUT_MODE ||
            pinModes[i] == GeneralGPIOConstants::INPUT_PULLUP_MODE) {
            // If a pin is in input mode, then read the value of the pin to sync state
            sendReadPacket(i);
        }
    }
}

void GeneralGPIOModule::sendSetModePacket(subDeviceIDConstant pin, payloadConstant mode) {
    // This function sends a set mode packet to the module
    ROIPackets::Packet packet;
    packet.setHostAddressOctet(transportAgent.getHostAddressOctet());
    packet.setClientAddressOctet(moduleOctet);
    packet.setSubDeviceID(pin);
    packet.setActionCode(GeneralGPIOConstants::SET_PIN_MODE);
    uint8_t data[1] = {mode};
    packet.setData(data, 1);
    transportAgent.queueGeneralPacket(packet);

    // Update the state of the pin
    pinModes[pin] = mode;
}

void GeneralGPIOModule::sendSetOutputPacket(subDeviceIDConstant pin, bool value) {
    // This function sends a set output packet to the module
    ROIPackets::Packet packet;
    packet.setHostAddressOctet(transportAgent.getHostAddressOctet());
    packet.setClientAddressOctet(moduleOctet);
    packet.setSubDeviceID(pin);
    packet.setActionCode(GeneralGPIOConstants::SET_OUTPUT);
    uint8_t data[2] = {value};
    packet.setData(data, 1);
    transportAgent.queueGeneralPacket(packet);

    // Update the state of the pin
    pinValues[pin] = value;
}

void GeneralGPIOModule::sendReadPacket(subDeviceIDConstant pin) {
    // This function sends a read packet to the module
    ROIPackets::Packet packet;
    packet.setHostAddressOctet(transportAgent.getHostAddressOctet());
    packet.setClientAddressOctet(moduleOctet);
    packet.setSubDeviceID(pin);
    packet.setActionCode(GeneralGPIOConstants::READ);
    transportAgent.queueGeneralPacket(packet);
}

bool GeneralGPIOModule::validatePin(subDeviceIDConstant pin) {
    if (!(pin < GeneralGPIOConstants::COUNT || pin == 8 || pin == 9)) {
        throw std::invalid_argument("Invalid pin subdevice ID Access " + std::to_string(pin));
    }
    return true;
}

/*----- Public Functions ------*/

GeneralGPIOModule::GeneralGPIOModule(uint8_t moduleOctet, TransportAgent& transportAgent)
    : transportAgent(transportAgent) {
    this->moduleOctet = moduleOctet;
    for (int i = 0; i < GeneralGPIOConstants::COUNT;
         i++) {  // Initialize all pins to input mode as default
        pinModes[i] = GeneralGPIOConstants::INPUT_MODE;
        pinValues[i] = 0;
    }
}

GeneralGPIOModule::~GeneralGPIOModule() {}

bool GeneralGPIOModule::pushState() {
    // This function pushes the state of the GPIO pins to the module
    for (int i = 0; i < GeneralGPIOConstants::COUNT; i++) {
        if (i > 7 && i < 10) {
            i = 9;  // skip empty slots... next real slot is 10
            continue;
        }
        sendSetModePacket(i, pinModes[i]);  // set the mode of all pins

        if (pinModes[i] == GeneralGPIOConstants::OUTPUT_MODE) {
            sendSetOutputPacket(i, pinValues[i]);  // set the output of all output pins
        }
    }

    return true;
}

bool GeneralGPIOModule::pullState() {
    // This function pulls the state of the GPIO pins from the module
    // Not implemented yet
    return false;
}

uint8_t GeneralGPIOModule::getOctet() {
    // This function returns the module octet of the module
    return moduleOctet;
}

void GeneralGPIOModule::setPinMode(subDeviceIDConstant pin, payloadConstant mode) {
    // This function sets the mode of a pin
    if (!validatePin(pin)) {
        return;  // probably redundant as exception will be thrown
    }
    sendSetModePacket(pin, mode);
}

bool GeneralGPIOModule::setOutput(subDeviceIDConstant pin, bool value) {
    // This function sets the output of a pin
    if (!validatePin(pin)) {
        return false;  // probably redundant as exception will be thrown
    }

    if (pinModes[pin] == GeneralGPIOConstants::OUTPUT_MODE) {
        sendSetOutputPacket(pin, value);
        return true;
    }
    return false;
}

bool GeneralGPIOModule::getDigitalInput(subDeviceIDConstant pin) {
    // This function gets the value of a digital pin
    if (!validatePin(pin)) {
        return false;  // probably redundant as exception will be thrown
    }

    if (pin < 10) {  // digital pin, always bool
        return (bool)pinValues[pin];
    } else {  // analog pin, return true if value is greater than 512
        if (pinModes[pin] == GeneralGPIOConstants::OUTPUT_MODE) {
            return (bool)pinValues[pin];
        } else {
            return (bool)(pinValues[pin] > 512);  // return true if value is greater than 512
        }
    }
}

uint16_t GeneralGPIOModule::getAnalogInput(subDeviceIDConstant pin) {
    // This function gets the value of pin where analog pins are given in analog format
    if (!validatePin(pin)) {
        return 0;  // probably redundant as exception will be thrown
    }

    return pinValues[pin];
}