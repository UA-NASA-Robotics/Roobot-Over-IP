#include "../ActuatorTypes.h"

class EncoderBase {
    private:
        bool _initialized = false;      // Bool to determine whether the encoder is initialized

        const int16_t _TRAVEL_DISTANCE; // Maximum length the encoder can detect (min to max)
        const float MM_PER_TICK;        // Unit conversion of mm per encoder tick

    public:
        virtual EncoderReading read() const = 0;

        virtual void load() const = 0;

        virtual void init() const = 0;

        virtual void reset() const = 0;
};