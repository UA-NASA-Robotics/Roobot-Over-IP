#include "floatCast.h"

float floatCast::uint8ArrayToFloat(uint8_t* array, uint16_t highByte, uint16_t lowByte) {
    if (abs(highByte - lowByte) != 3) return 0;  // Check if the spacing is correct
    uint32_t u = 0;

    if (highByte < lowByte) {  // big endian
        for (uint16_t i = 0; i < 4; i++) {
            u <<= 8;
            u |= array[highByte + i];
        }
    } else {  // little endian
        for (uint16_t i = 0; i < 4; i++) {
            u <<= 8;
            u |= array[lowByte + i];
        }
    }

    return *reinterpret_cast<float*>(&u);  // reinterpret the pointer to a uint32_t as a pointer to
                                           // a float and then dereference it.
}

float floatCast::uint8sToFloat(uint8_t highByte, uint8_t highMidByte, uint8_t lowMidByte,
                               uint8_t lowByte) {
    uint32_t u = 0;
    u |= highByte;
    u <<= 8;
    u |= highMidByte;
    u <<= 8;
    u |= lowMidByte;
    u <<= 8;
    u |= lowByte;

    return *reinterpret_cast<float*>(&u);
}

bool floatCast::floatToUint8Array(float f, uint8_t* array, uint16_t highByte, uint16_t lowByte) {
    if (abs(highByte - lowByte) != 3) return false;  // Check if the spacing is correct
    uint32_t u = *reinterpret_cast<uint32_t*>(&f);

    if (highByte < lowByte) {  // big endian
        for (uint16_t i = 0; i < 4; i++) {
            array[highByte + i] = u >> (8 * (3 - i));
        }
    } else {  // little endian
        for (uint16_t i = 0; i < 4; i++) {
            array[lowByte + i] = u >> (8 * (3 - i));
        }
    }

    return true;
}