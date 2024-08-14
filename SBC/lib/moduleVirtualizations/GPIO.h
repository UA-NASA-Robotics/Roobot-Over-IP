#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#include "../transportAgent.h"
#include "base.h"

namespace GeneralGPIOModule {

class GeneralGPIOModule : public BaseModule {
   private:
    uint8_t moduleOctet;  // The module octet

    TransportAgent& transportAgent;  // The transport agent

    uint8_t pinModes[GeneralGPIOConstants::COUNT];    // The state of the GPIO pins
    uint16_t pinValues[GeneralGPIOConstants::COUNT];  // The value of the GPIO pins

    void ResponseCallback(ROIPackets::Packet packet);
    void MaintainState();

   public:
    GeneralGPIOModule(uint8_t moduleOctet, TransportAgent& transportAgent);
    ~GeneralGPIOModule();

    bool PushState();
    bool PullState();

    // GPIO specific functions

    void setPinMode(subDeviceIDConstant pin, payloadConstant mode);

    void setOutput(subDeviceIDConstant pin, bool value);

    void getInput(subDeviceIDConstant pin);
};
}  // namespace GeneralGPIOModule

#endif