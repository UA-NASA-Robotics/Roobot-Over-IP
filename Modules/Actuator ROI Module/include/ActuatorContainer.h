#ifndef ACTUATOR_CONTAINER_H
#define ACTUATOR_CONTAINER_H

#include "../../../lib/Packet.h"
#include "../../../lib/floatCast.h"
#include "Actuator.h"
#include <stdint.h>

template<int N>
class ActuatorContainer {
    private:
        bool _initialized;      // Flag to determine if container was initialized
        const uint8_t _SIZE;    // Container size

        Actuator* _acts[N];     // Array of pointer to actuators

    public:
        /**
        * @brief ActuatorContainer class constructor
        */
        ActuatorContainer();

        /**
        * @brief ActuatorContainer class deconstructor
        */
        ~ActuatorContainer();

        /**
        * @brief Append an actuator to the end of the container's array
        * 
        * @param act    An actuator class object
        */
        void append(Actuator& act);

        /**
        * @brief ActuatorContainer class constructor
        * 
        * @param act    An actuator class object
        * @param index  The desired index the actuator should be placed
        */
        void set(Actuator& act, uint8_t index);

        /**
        * @brief Initialize each connected actuator
        */
        void init();

        /**
        * @brief Tick each connected actuator
        */
        void tick();

        /**
        * @brief Handle a general ROI packet
        * 
        * @param packet The provided ROI packet
        * 
        * @return A return ROI packet
        */
        ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet& packet);
};



template<int N>
ActuatorContainer<N>::ActuatorContainer() : _SIZE(N) {
    _initialized = false;
    
    for (int i = 0; i < N; i++)
        _acts[i] = nullptr;
}

template<int N>
ActuatorContainer<N>::~ActuatorContainer() {
    for (int i = 0; i < N; i++)
        delete _acts[i];
}

template<int N>
void ActuatorContainer<N>::append(Actuator& act) {
    //assert(_initialized);
    
    for (int i = 0; i < N; i++) {
        if (_acts[N] == nullptr) {
            _acts[N] = &act;
            return;
        }
    }
}

template<int N>
void ActuatorContainer<N>::set(Actuator& act, uint8_t index) {
    //assert(_initialized && index < N);
    
    if (index < _SIZE)
        _acts[index] = &act;
}

template<int N>
void ActuatorContainer<N>::init() {
    for (Actuator* act : _acts)
        if (act) act->init();
    
    _initialized = true;
}

template<int N>
void ActuatorContainer<N>::tick() {
    for (Actuator* act : _acts) {
        if (act) act->tick();
    }
}

template<int N>
ROIPackets::Packet ActuatorContainer<N>::handleGeneralPacket(ROIPackets::Packet& packet) {
    using namespace ActuatorConstants;
    uint16_t id = packet.getSubDeviceID();
    Actuator* act = _acts[id];                              // Actuator requested

    // OOB Checking
    if (id < N && act != nullptr) {
        return packet.swapReply().setData(0);
    }

    // Create a buffer to store the data from the packet
    uint8_t generalBuffer[28];
    
    // Get the payload from the packet
    packet.getData(generalBuffer, ROIConstants::ROIMAXPACKETPAYLOAD);  

    // Operation management
    switch(packet.getActionCode()) {
    case (SET_RELATIVE_LENGTH): {
        uint16_t length = (generalBuffer[0] << 8) | generalBuffer[1];
        act->setRelativeLength(length);
        return packet.swapReply().setData(0);
    }
    case (SET_ABSOLUTE_LENGTH): {
        uint16_t length = (generalBuffer[0] << 8) | generalBuffer[1];
        act->setAbsoluteLength(length);
        return packet.swapReply().setData(0);
    }
    case (GET_TARGET_LENGTH):
        return packet.swapReply().setData(0);

    case (GET_CURRENT_LENGTH):
        return packet.swapReply().setData(0);

    case (SET_VELOCITY): {
        float velocity = floatCast::toFloat(generalBuffer, 0, 3);
        act->setVelocity(velocity);
        return packet.swapReply().setData(0);
    }

    case (GET_TARGET_VELOCITY):
        return packet.swapReply().setData(0);

    case (GET_CURRENT_VELOCITY):
        return packet.swapReply().setData(0);

    case (SET_HOME_MAX):
        act->home(true);
        return packet.swapReply().setData(0);

    case (SET_HOME_MIN):
        act->home(false);
        return packet.swapReply().setData(0);

    case (GET_HOMED):
        return packet.swapReply().setData(0);

    case (SET_CONTROL):
        act->setControlMode(generalBuffer[0]);
        return packet.swapReply().setData(0);

    case (GET_CONTROL):
        return packet.swapReply().setData(0);

    case (SET_SPEED_PID):
        return packet.swapReply().setData(0);

    case (SET_LENGTH_PID):
        return packet.swapReply().setData(0);

    case (GET_SPEED_PID):
        return packet.swapReply().setData(0);

    case (GET_LENGTH_PID):
        return packet.swapReply().setData(0);
    };
}


#endif