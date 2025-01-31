#include "EncoderDriverBase.h"

class FirgelliEncoder : protected EncoderDriverBase {
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
         * @brief Returns the position of an encoder reading in millimeters
         * 
         * @param reading   An encoder reading scruct
         */
        static uint16_t toMM(EncoderReading reading);
        
        /**
         * @brief Returns the position of an encoder reading's position in millimeters
         * 
         * @param rotations A number of encoder rotations
         */
        static uint16_t toMM(uint16_t rotations);

        /**
         * @brief Constructor for FirgelliEncoder class
         */
        FirgelliEncoder(uint8_t load, uint8_t clk, uint8_t shft, uint8_t clr);

        /**
         * @brief Initialize this encoder
         */
        void init() override;
};