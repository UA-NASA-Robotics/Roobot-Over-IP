#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "../include/Encoders/EncoderBase.h"
#include "../include/ActuatorPid.h"
#include "ActuatorTypes.h"

// When homing, pass the maximum traversal length as a constructor constant, then home either max/min length

class Actuator {
    private:
        const uint16_t _TRAVEL_LIMIT;   // The amount of distance the actuator can move between it's
                                        // minimum and maximum limits

        ActuatorPins _PINS;             // Actuator's Arduino pins
        ActuatorPid _pid;               // Actuator's PID controller
        EncoderBase* _enc;              // External encoder

    public:
        friend class ActuatorContainer;
        
        Actuator(ActuatorPins pins = {}, uint16_t travel_limit = 0, PidTuning tuning);
        Actuator(ActuatorPins pins = {}, uint16_t travel_limit = 0);
        
        uint16_t getLastRead();
        void init();
        void loop();
};


#endif