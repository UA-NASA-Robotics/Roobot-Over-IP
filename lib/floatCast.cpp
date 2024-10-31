#include "floatCast.h"

float floatCast::toFloat(uint8_t* array, uint16_t highByte, uint16_t lowByte) {
    if (floatCast::absoluteValue(highByte - lowByte) != 3)
        return 0;  // Check if the spacing is correct
    uint32_t u = 0;

    if (highByte < lowByte) {  // big endian
        toFloat(array[highByte], array[highByte + 1], array[highByte + 2], array[highByte + 3]);
    } else {  // little endian
        toFloat(array[lowByte], array[lowByte + 1], array[lowByte + 2], array[lowByte + 3]);
    }
}

float floatCast::toFloat(uint8_t highByte, uint8_t highMidByte, uint8_t lowMidByte,
                         uint8_t lowByte) {
    uint32_t u = 0;
    u |= highByte;
    u <<= 8;
    u |= highMidByte;
    u <<= 8;
    u |= lowMidByte;
    u <<= 8;
    u |= lowByte;

    return toFloat(u);
}

float floatCast::toFloat(uint32_t bigEndian) {
    return *reinterpret_cast<float*>(&bigEndian);
}  // reinterpret the pointer to a uint32_t as a pointer to a float and then dereference it.

bool floatCast::floatToUint8Array(float f, uint8_t* array, uint16_t highByte, uint16_t lowByte) {
    if (floatCast::absoluteValue(highByte - lowByte) != 3)
        return false;  // Check if the spacing is correct
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

float floatCast::absoluteValue(float f) { return f < 0 ? -f : f; }