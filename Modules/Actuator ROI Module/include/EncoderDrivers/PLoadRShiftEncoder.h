#include "EncoderDriverBase.h"

/*
    Parallel-Load Right-Shift Encoder
    - Designed to be compatible with custom circuitry
      for the 2024-25 season on Firgelli actuators

    Design:
    - Compatible with quad encoders using Hall effect sensors
    - Counters on the hardware are asynchronously updated
      with the actuator's internal rotations
    - To read the counter value, the count must be parallel
      loaded into a "16-bit" right-shift register

    Andrew Piunno (ajp235),
    2/26/2025
*/

class PLoadRShiftEncoder : public EncoderDriverBase {
    private:
        const uint8_t _LOAD, _CLK, _SHFT, _CLR; // Encoder pins

        /**
         * @brief Load the current encoder value in hardware
         */
        void _load() override;

        /**
         * @brief Read a loaded value from the encoder hardware
         */
        void _read() override;

    public:
        // Unit conversion of encoder ticks per millimeter
        constexpr static const float TICKS_PER_MM = 17.4;

        /**
         * @brief Constructor for PLoadRShiftEncoder class
         */
        PLoadRShiftEncoder(uint8_t load, uint8_t clk, uint8_t shft, uint8_t clr);
        
        /**
         * @brief Clear the encoder's counter
         */
        void clear();

        /**
         * @brief Initialize this encoder
         */
        void init() override;

        /**
         * @brief Update the homed length of the encoder
         * 
         * @param length    The homed length of the encoder in mm
         */
        void home(uint16_t length) override;
};