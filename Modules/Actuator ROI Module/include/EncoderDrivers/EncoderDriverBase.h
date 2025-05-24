#ifndef ENCODERBASE_H
#define ENCODERBASE_H

#include <stdint.h>

// Container to hold the value of an encoder read
struct EncoderReading {
    int16_t length = 0;
    int32_t time = 0;
};

// Virtual class for actuator encoders
class EncoderDriverBase {
   protected:
    bool _initialized = false;  // Flag to determine whether the encoder is initialized
    bool _loaded = false;       // Flag to determine if there is data the encoder can read

    uint16_t _homed_length;  // Length the actuator was homed at

    EncoderReading _prev_read;  // 2nd-most recent encoder read
    EncoderReading _cur_read;   // Most recent encoder read

    /**
     * @brief Load the current encoder value in hardware
     */
    virtual void _load() = 0;

    /**
     * @brief Read a loaded value from the encoder hardware
     */
    virtual void _read() = 0;

   public:
    /**
     * @brief Initialize this encoder
     */
    virtual void init() = 0;

    /**
     * @brief Run a load/read cycle on the encoder
     */
    void tick();

    /**
     * @brief Update the homed length of the encoder
     *
     * @param length    The homed length of the encoder in mm
     */
    virtual void home(uint16_t length);

    /**
     * @brief Get the most recent encoder reading
     *
     * @return A struct containing a int16_t length in encoder ticks
     *         and a int32_t time in milliseconds
     */
    EncoderReading value();

    /**
     * @brief Get the average RPM of the encoder from the last two reads
     *
     * @return The average RPM of the encoder from the last two reads
     */
    float velocity();
};

#endif