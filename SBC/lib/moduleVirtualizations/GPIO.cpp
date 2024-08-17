#include "GPIO.h"

/*----- Private Functions -----*/

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

void GeneralGPIOModule::GeneralGPIOModule::SendSetModePacket(subDeviceIDConstant pin,
                                                             payloadConstant mode) {
    // This function sends a set mode packet to the module
    ROIPackets::Packet packet;
    packet.setHostAddressOctet(transportAgent.getHostAddressOctet());
    packet.setClientAddressOctet(moduleOctet);
    packet.setSubDeviceID(pin);
    packet.setActionCode(GeneralGPIOConstants::SET_PIN_MODE);
    packet.setPayload(mode);
    transportAgent.queueGeneralPacket(packet);

    // Update the state of the pin
    pinModes[pin] = mode;
}

void GeneralGPIOModule::GeneralGPIOModule::SendSetOutputPacket(subDeviceIDConstant pin,
                                                               bool value) {
    // This function sends a set output packet to the module
    ROIPackets::Packet packet;
    packet.setHostAddressOctet(transportAgent.getHostAddressOctet());
    packet.setClientAddressOctet(moduleOctet);
    packet.setSubDeviceID(pin);
    packet.setActionCode(GeneralGPIOConstants::SET_OUTPUT);
    packet.setPayload(value);
    transportAgent.queueGeneralPacket(packet);

    // Update the state of the pin
    pinValues[pin] = value;
}

void GeneralGPIOModule::GeneralGPIOModule::SendReadPacket(subDeviceIDConstant pin) {
    // This function sends a read packet to the module
    ROIPackets::Packet packet;
    packet.setHostAddressOctet(transportAgent.getHostAddressOctet());
    packet.setClientAddressOctet(moduleOctet);
    packet.setSubDeviceID(pin);
    packet.setActionCode(GeneralGPIOConstants::READ);
    transportAgent.queueGeneralPacket(packet);
}

/*----- Public Functions ------*/

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

void GeneralGPIOModule::GeneralGPIOModule::PushState() {
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
}

bool GeneralGPIOModule::GeneralGPIOModule::PullState() {
    // This function pulls the state of the GPIO pins from the module
    // Not implemented yet
    return false;
}

void GeneralGPIOModule::GeneralGPIOModule::SetPinMode(subDeviceIDConstant pin,
                                                      payloadConstant mode) {
    // This function sets the mode of a pin
    if (pin < GeneralGPIOConstants::COUNT && pin != 8 && pin != 9) {
        sendSetModePacket(pin, mode);
    } else {
        RaiseException("Invalid pin number");  // Raising errors is bad, but this is essentially an
                                               // unacceptable error
    }
}

bool GeneralGPIOModule::GeneralGPIOModule::SetOutput(subDeviceIDConstant pin, bool value) {
    // This function sets the output of a pin
    if (pin >= GeneralGPIOConstants::COUNT || pin == 8 || pin == 9) {
        RaiseException("Invalid pin number");  // Raising errors is bad, but this is essentially an
        // unacceptable error
        return false;
    }

    if (pinModes[pin] == GeneralGPIOConstants::OUTPUT_MODE) {
        sendSetOutputPacket(pin, value);
        return true;
    }
    return false;
}

bool GeneralGPIOModule::GeneralGPIOModule::GetDigitalInput(subDeviceIDConstant pin) {
    // This function gets the value of a digital pin
    if (pin >= GeneralGPIOConstants::COUNT || pin == 8 || pin == 9) {
        RaiseException("Invalid pin number");  // Raising errors is bad, but this is essentially an
        // unacceptable error
        return false;
    }

    if (pin < 10) {  // digital pin, always bool
        return (bool)pinValues[pin];
    } else {  // analog pin, return true if value is greater than 512
        if (pinMode[pin] == GeneralGPIOConstants::OUTPUT_MODE) {
            return (bool)pinValues[pin];
        } else {
            return (bool)(pinValues[pin] > 512);  // return true if value is greater than 512
        }
    }
}

uint16_t GeneralGPIOModule::GeneralGPIOModule::GetAnalogInput(subDeviceIDConstant pin) {
    // This function gets the value of pin where analog pins are given in analog format
    if (pin >= GeneralGPIOConstants::COUNT || pin == 8 || pin == 9) {
        RaiseException("Invalid pin number");  // Raising errors is bad, but this is essentially an
        // unacceptable error
        return 0;
    }

    return pinValues[pin];
}