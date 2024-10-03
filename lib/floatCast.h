#ifndef FLOATCAST_H
#define FLOATCAST_H

#include <stdint.h>

namespace floatCast {
/**
 * @brief Unions an array of 4 bytes into a float. Based on the highByte and lowByte index it will
 * handle big or little endian
 *
 * @param array , the array containing bytes to convert
 * @param highByte , the index of the high byte in the array
 * @param lowByte , the index of the low byte in the array
 * @return float, the float value of the array
 */
float uint8ArrayToFloat(uint8_t* array, uint16_t highByte, uint16_t lowByte);

/**
 * @brief Unions 4 bytes into a float.
 *
 * @param highByte
 * @param highMidByte
 * @param lowMidByte
 * @param lowByte
 * @return float
 */
float uint8sToFloat(uint8_t highByte, uint8_t highMidByte, uint8_t lowMidByte, uint8_t lowByte);

/**
 * @brief Separates a float into 4 bytes for sending over a network.
 *
 * @param f , float to separate
 * @param array , array to store the bytes
 * @param highByte , the index of the high byte in the array
 * @param lowByte , the index of the low byte in the array
 * @return true, if the conversion was successful
 * @return false, if the conversion was unsuccessful
 */
bool floatToUint8Array(float f, uint8_t* array, uint16_t highByte, uint16_t lowByte);

/**
 * @brief Returns the absolute value of a float
 *
 * @param f
 * @return float
 */
float absoluteValue(float f);

}  // namespace floatCast

#endif