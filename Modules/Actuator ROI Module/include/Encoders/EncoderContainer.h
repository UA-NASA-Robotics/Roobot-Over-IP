#include "EncoderBase.h"

class EncoderContainer {
    private:
        EncoderBase* _encoders;

    public:
        EncoderContainer();

        EncoderBase operator[](uint8_t index);

        void initAll();

        void resetAll();

        void loadAll();
};