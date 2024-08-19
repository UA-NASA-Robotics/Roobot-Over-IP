#include "../lib/moduleVirtualizations/GPIO.h"
#include "../lib/transportAgent.h"
#include "singleton.h"

class WhiskeyRobot : public Singleton {
   private:
    TransportAgent transportAgent;
    GeneralGPIOModule gpioModule1;

    WhiskeyRobot() : gpioModule1(231, transportAgent) { transportAgent.pushModule(&gpioModule1); }

   public:
};