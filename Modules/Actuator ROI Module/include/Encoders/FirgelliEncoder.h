#include "EncoderDriverBase.h"

class FirgelliEncoder : public EncoderDriverBase {
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
         * @brief Constructor for FirgelliEncoder class
         */
        FirgelliEncoder(uint8_t load, uint8_t clk, uint8_t shft, uint8_t clr);
        
        /**
         * @brief Clear the encoder's counter
         */
        void clear();

        /**
         * @brief Initialize this encoder
         */
        void init() override;
};