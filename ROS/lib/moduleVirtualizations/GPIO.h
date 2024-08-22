#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#include "../transportAgent.h"
#include "base.h"

class GeneralGPIOModule : public BaseModule {
   private:
    TransportAgent& transportAgent;  // The transport agent

    uint8_t pinModes[GeneralGPIOConstants::COUNT];    // The state of the GPIO pins
    uint16_t pinValues[GeneralGPIOConstants::COUNT];  // The value of the GPIO pins

    void responseCallback(ROIPackets::Packet packet);
    void maintainState();

    void sendSetModePacket(
        subDeviceIDConstant pin,
        payloadConstant mode);  // these are essentially API calls, they issue packets.
    void sendSetOutputPacket(subDeviceIDConstant pin, bool value);  // internal use only
    void sendReadPacket(subDeviceIDConstant pin);

    bool validatePin(subDeviceIDConstant pin);  // validates a pin

   public:
    GeneralGPIOModule(uint8_t moduleOctet, TransportAgent& transportAgent);
    ~GeneralGPIOModule();

    bool pushState();  // pushes the state of the GPIO pins to the module
    bool pullState();  // pulls the state of the GPIO pins from the module (not implemented yet)

    uint8_t getOctet();  // returns the module octet of the module

    // GPIO specific functions

    void setPinMode(subDeviceIDConstant pin, payloadConstant mode);  // sets a pin to a mode

    bool setOutput(subDeviceIDConstant pin,
                   bool value);  // sets an output pin to a value, true if successful

    bool getDigitalInput(
        subDeviceIDConstant pin);  // gets the value of a pin, true if high (works for all pins)

    uint16_t getAnalogInput(
        subDeviceIDConstant pin);  // gets the value of a pin, 0-1 if digital pin, 0-1023 if analog
                                   // pin (works for all pins)
};

#endif