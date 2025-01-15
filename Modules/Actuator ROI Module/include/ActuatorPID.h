#pragma once
#include <stdint.h>


// Container to hold the kp, ki, and kd tuning parameters for an actuators PID controller
struct PidTuning {
    PidTuning(float kp = 0, float ki = 0, float kd = 0);

    float kp;
    float ki;
    float kd;
};

// Class to control the PID loop for an actuator
class ActuatorPid {
    private:
        float _i;               // Cummulative sum for integral error
        float _prev_p;          // Stored last position to calculate derivative

        float _velocity;        // The target velocity the controller is aiming for
        float _max_velocity;    // The maximum velocity the controller can use (always positive)
        uint16_t _target_pos;   // The target position the controller is aiming for
        
        uint32_t _prev_time;    // Stored time since last PID loop cycle

        PidTuning _tuning;      // Proportion, integral, and derivative constants

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
         * @brief Tunes the PID controller with the provided constants
         * 
         * @param tuning  A PidTuning struct containing tuning constants
         */
        void setTuning(PidTuning tuning);

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
         * @return The current target velocity for the PID controller (as a percent)
         */
        float getVelocity() {return _velocity; };

        /**
         * @brief Returns an array containing the PID constants as an array
         * 
         * @return An std::array of 3 floats: kp, ki, and kd
         */
        PidTuning getTuning() { return _tuning; };

        /**
         * @brief Run a single cycle for the PID controller, 
         * 
         * Returns AXIS_STATE_UNDEFINED in case of a communication error.
         */
        void loop();
};
