#ifndef ACTUATOR_CONTAINER_H
#define ACTUATOR_CONTAINER_H

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


#endif