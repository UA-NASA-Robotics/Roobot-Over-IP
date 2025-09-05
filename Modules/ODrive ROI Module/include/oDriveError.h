#ifndef ODRIVEERROR_H
#define ODRIVEERROR_H

#include <ODriveEnums.h>
#include <stdint.h>

namespace oDriveError {
/**
 * @brief Returns true if the ODRIVE system is in an operable state with error
 *
 * @param error
 * @return true, ODrive is working with error
 * @return false, ODrive is not working with error
 */
bool errorIsOperable(uint32_t error);

/**
 * @brief Returns true if error is light (easy to fix like over current fault) and should auto clear
 *
 * @param error
 * @return true
 * @return false
 */
bool errorShouldAutoClear(uint32_t error);

};  // namespace oDriveError

#endif