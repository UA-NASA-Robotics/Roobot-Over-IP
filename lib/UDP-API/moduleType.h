#ifndef MODULE_TYPE_H
#define MODULE_TYPE_H

#include <stdint.h>

namespace moduleTypesConstants {

typedef uint16_t moduleTypeConstant;

/*--------- Module ID Codes ----------------*/
// We are skipping 0 and 1 as they may be used for fail or error codes
constexpr moduleTypeConstant MASTER_SBC =
    2;  // The MASTER_SBC module returns a 2 as it's id in a ping
constexpr moduleTypeConstant GENERAL_GPIO =
    3;                                     // A generalGPIO module returns a 3 as it's id in a ping
constexpr moduleTypeConstant O_DRIVE = 4;  // An ODrive module returns a 4 as it's id in a ping
constexpr moduleTypeConstant ACTUATOR = 5;
}  // namespace moduleTypesConstants

#endif  // MODULE_TYPE_H