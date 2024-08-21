#ifndef HEADLIGHT_H
#define HEADLIGHT_H

#include "GPIO.h"

// Makes a single headlight alias of a general GPIO module

class HeadLight : public GeneralGPIOModule {
   private:
    subDeviceIDConstant pin = 0;
    bool initialized = false;
    bool on = false;

    void init();

   public:
    HeadLight(uint8_t hostOctet, TransportAgent& transportAgent, subDeviceIDConstant lightPin);

    void setOn(bool state);

    bool isOn();
};

#endif