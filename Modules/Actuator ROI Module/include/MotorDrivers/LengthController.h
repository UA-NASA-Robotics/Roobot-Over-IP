#ifndef LENGTH_CONTROLLER_H
#define LENGTH_CONTROLLER_H

#include "../EncoderDrivers/EncoderDriverBase.h"
#include <stdint.h>

class LengthController {
    private:
        bool _is_binary;        // Flag to determine if the motor should be variable speed (false) or one speed (true)

        int16_t _target_len;    // The target actuator length
        float _target_vel;      // The target velocity relative to the target length

    public:
        /**
         * @brief LengthController class constructor
         */
        LengthController();

        /**
         * @brief Set the binary state of the motor connected to this LengthController
         * 
         * @param is_binary Bool to determine if the motor is binary or not
         */
        void isBinary(bool is_binary);
        
        /**
         * @brief Set the target length of the length controller
         * 
         * @param target_len    The desired target length of the length controller
         */
        void setLength(int16_t target_len);

        /**
         * @brief Return the calculated target velocity of the controller
         */
        float velocity();

        /**
         * @brief Calculate the target velocity from an encoder reading
         * 
         * @param reading   The most recent encoder reading
         */
        void tick(EncoderReading reading);
};

#endif