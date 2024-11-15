#include <stdint.h>
#pragma once

class ActuatorPid {
    private:
        float _kp, _ki, _kd;    // Proportion, integral, and derivative constants
        float _i;               // Cummulative sum for integral error
        float _prev_p;          // Stored last position to calculate derivative

        float _velocity;        // The target velocity the controller is aiming for
        float _max_velocity;    // The maximum velocity the controller can use (always positive)
        uint16_t _target_pos;   // The target position the controller is aiming for
        
        uint32_t _prev_time;    // Stored time since last PID loop cycle

    public:
        /**
         * @brief       Constructor for the ActuatorPid class
         *
         * @param _kp   Proportionality constant
         * @param _ki   Integration constant
         * @param _kd   Differentiation constant
         */
        ActuatorPid(float kp, float ki, float kd);
        
        /**
         * @brief Tunes the PID controller with the provided constants
         * 
         * @param kp    Proportionality constant
         * @param ki    Integration constant
         * @param kd    Differentiation constant
         */
        void setTuning(float kp, float ki, float kd);

        /**
         * @brief Sets a target position and a max velocity duty cycle for the actuator
         * 
         * @param position      The uint16_t target position for the actuator
         * @param max_velocity  The maximum velocity for this target (as a percent)
         */
        void setTarget(uint16_t position, float max_velocity);

        /**
         * @brief Returns the current target position for the PID controller
         * 
         * @return The current target position for the PID controller
         */
        uint16_t getTargetPosition() { return _target_pos; };

        /**
         * @brief Returns the current target velocity for the PID controller (as a percent)
         * 
         * @return The current target velocit for the PID controller (as a percent)
         */
        float getVelocity() {return _velocity; };

        /**
         * @brief Returns an array containing the PID constants as an array
         * 
         * @return Pointer to the first index of an array of 3 floats: kp, ki, and kd
         */
        float* getTuning() { return (float[]){_kp, _ki, _kd}; };

        /**
         * @brief Run a single cycle for the PID controller, 
         * 
         * Returns AXIS_STATE_UNDEFINED in case of a communication error.
         */
        void loop();
};