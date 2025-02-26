#ifndef ACTUATOR_CONTAINER_H
#define ACTUATOR_CONTAINER_H

#include <stdint.h>

#include "../../../lib/Packet.h"
#include "../../../lib/floatCast.h"
#include "Actuator.h"

template <int N>
class ActuatorContainer {
   private:
    bool _initialized;    // Flag to determine if container was initialized
    const uint8_t _SIZE;  // Container size

    Actuator* _acts[N];  // Array of pointer to actuators

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

template <int N>
ActuatorContainer<N>::ActuatorContainer() : _SIZE(N) {
    _initialized = false;

    for (int i = 0; i < N; i++) _acts[i] = nullptr;
}

template <int N>
ActuatorContainer<N>::~ActuatorContainer() {
    for (int i = 0; i < N; i++) delete _acts[i];
}

template <int N>
void ActuatorContainer<N>::append(Actuator& act) {
    // assert(_initialized);

    for (int i = 0; i < N; i++) {
        if (_acts[i] == nullptr) {
            _acts[i] = &act;
            return;
        }
    }
}

template <int N>
void ActuatorContainer<N>::set(Actuator& act, uint8_t index) {
    // assert(_initialized && index < N);

    if (index < _SIZE) _acts[index] = &act;
}

template <int N>
void ActuatorContainer<N>::init() {
    for (Actuator* act : _acts)
        if (act) act->init();

    _initialized = true;
}

template <int N>
void ActuatorContainer<N>::tick() {
    for (Actuator* act : _acts)
        if (act) act->tick();
}

template <int N>
ROIPackets::Packet ActuatorContainer<N>::handleGeneralPacket(ROIPackets::Packet& packet) {
    using namespace ActuatorConstants;
    uint16_t id = packet.getSubDeviceID();
    Actuator* act = _acts[id];  // Actuator requested

    ROIPackets::Packet reply_packet = packet.swapReply();
    reply_packet.setData(0);

    // OOB Checking
    if (id >= N || act == nullptr) {
        return reply_packet;
    }

    // Create a buffer to store the data from the packet
    uint8_t generalBuffer[ROIConstants::ROI_MAX_PACKET_PAYLOAD]{0};

    // Get the payload from the packet
    packet.getData(generalBuffer, ROIConstants::ROI_MAX_PACKET_PAYLOAD);

    // Serial.println(packet.getSubDeviceID());
    // Serial.println(packet.getActionCode());
    // for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {
    //     Serial.println(generalBuffer[i]);
    // }
    //Serial.println("\n\n\n");

    // Operation management
    switch (packet.getActionCode()) {
        case (SET_RELATIVE_LENGTH): {
            uint16_t length = (generalBuffer[0] << 8) | generalBuffer[1];
            act->setRelativeLength(length);
            return reply_packet;
        }
        case (SET_ABSOLUTE_LENGTH): {
            uint16_t length = (generalBuffer[0] << 8) | generalBuffer[1];
            act->setAbsoluteLength(length);
            return reply_packet;
        }
        case (GET_TARGET_LENGTH):
            return reply_packet;

        case (GET_CURRENT_LENGTH):
            return reply_packet;

    case (SET_VELOCITY): {
        float velocity = floatCast::toFloat(generalBuffer, 0, 3);
        act->setVelocity(velocity);
        return reply_packet;
    }

        case (GET_TARGET_VELOCITY):
            return reply_packet;

        case (GET_CURRENT_VELOCITY):
            return reply_packet;

    case (GET_HOMED):
        return reply_packet;

        case (SET_CONTROL):
            act->setControlMode(generalBuffer[0]);
            return reply_packet;

        case (GET_CONTROL):
            return reply_packet;

        case (SET_SPEED_PID):
            return reply_packet;

        case (SET_LENGTH_PID):
            return reply_packet;

        case (GET_SPEED_PID):
            return reply_packet;

        case (GET_LENGTH_PID):
            return reply_packet;
    };

    return reply_packet;
}

#endif