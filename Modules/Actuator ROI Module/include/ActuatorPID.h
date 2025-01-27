#ifndef ACTUATOR_PID_H
#define ACTUATOR_PID_H

#include "ActuatorTypes.h"

// Class to control the PID loop for an actuator
class ActuatorPid {
    private:
        bool _paused;           // Flag determining if the actuator is paused

        float _i;               // Cummulative sum for integral error
        float _prev_p;          // Stored last position to calculate derivative

        float _speed;           // The target speed the controller is aiming for
        float _max_speed;       // The maximum speed the controller can use (always positive)
        uint16_t _target_pos;   // The target position the controller is aiming for
        
        uint32_t _prev_time;    // Stored time since last PID loop cycle

        PidTuning _tuning;      // Proportion, integral, and derivative constants

    public:
        friend class Actuator;  // Allow the actuator to access privated members

        /**
         * @brief       Default constructor for the ActuatorPid class
         *
         * @param _kp   Proportionality constant
         * @param _ki   Integration constant
         * @param _kd   Differentiation constant
         */
        ActuatorPid(float kp = 0, float ki = 0, float kd = 0);

        /**
         * @brief       Overloaded constructor for the ActuatorPid class
         *
         * @param _kp   PidTuning struct containing values for kp, ki, and kd
         */
        ActuatorPid(PidTuning tuning);
        
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
         * @brief Sets a target position and a max speed duty cycle for the actuator
         * 
         * @param position  The uint16_t target position for the actuator
         * @param max_speed The maximum speed for this target (as a percent)
         */
        void setTarget(uint16_t position, float max_speed = 1);

        /**
         * @brief Returns the current target position for the PID controller
         * 
         * @return The current target position for the PID controller
         */
        uint16_t getTargetPosition() { return _target_pos; };

        /**
         * @brief Returns the current target speed for the PID controller (as a percent)
         * 
         * @return The current target speed for the PID controller (as a percent)
         */
        float getSpeed() {return _speed; };

        /**
         * @brief Returns an array containing the PID constants as an array
         * 
         * @return An std::array of 3 floats: kp, ki, and kd
         */
        PidTuning getTuning() { return _tuning; };

        /**
         * @brief Run a single cycle for the PID controller
         * 
         * @param cur_pos The last read encoder position
         * 
         * Returns AXIS_STATE_UNDEFINED in case of a communication error.
         */
        void loop(EncoderReading reading);

        /**
         * @brief Pause the actuator's movement by forcing the speed to zero
         */
        void pause();

        /**
         * @brief Resume the actuator's movement after being paused 
         */
        void resume();

        /**
         * @brief Resets the actuators by pulling them to their minimum position
         */
        void home();
};


#endif