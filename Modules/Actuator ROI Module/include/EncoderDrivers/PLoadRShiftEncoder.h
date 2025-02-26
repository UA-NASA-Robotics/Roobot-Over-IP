#include "EncoderDriverBase.h"

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