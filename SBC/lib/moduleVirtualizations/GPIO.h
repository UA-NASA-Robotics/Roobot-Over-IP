#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#include "base.h"

class GeneralGPIOModule : public BaseModule {
   private:
    uint8_t pinModes[GeneralGPIOConstants::COUNT];    // The state of the GPIO pins
    uint16_t pinValues[GeneralGPIOConstants::COUNT];  // The value of the GPIO pins

    void ResponseCallback(ROIPackets::Packet packet);
    void MaintainState();

   public:
    GeneralGPIOModule(uint8_t moduleOctet);
    ~GeneralGPIOModule();

    bool PushState();
    bool PullState();

    // GPIO specific functions

    void setPinMode(subDeviceIDConstant pin, payloadConstant mode);

    void setOutput(subDeviceIDConstant pin, bool value);

    void getInput(subDeviceIDConstant pin);
};

#endif