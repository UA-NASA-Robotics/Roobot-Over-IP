#include "headLight.h"

void HeadLight::init() {
    if (!initialized) {
        setPinMode(pin, GeneralGPIOConstants::OUTPUT_MODE);
        initialized = true;
    }
}

/*---- Public ----*/
HeadLight::HeadLight(uint8_t hostOctet, TransportAgent& transportAgent,
                     subDeviceIDConstant lightPin)
    : GeneralGPIOModule(hostOctet, transportAgent), pin(lightPin) {}

void HeadLight::setOn(bool state) {
    init();
    on = state;
    setOutput(pin, state);
}

bool HeadLight::isOn() { return on; }
