#ifndef MACGEN_H
#define MACGEN_H

#ifdef __AVR__
#include <Arduino.h>
#else
//  For non-AVR systems
#endif

#include <stdint.h>

// Set the default debug mode for the macGen
#ifndef DEBUG
#define DEBUG false
#endif

namespace macGen {

constexpr uint8_t compileTime[] = __TIME__;  // Seed A for the MAC address generation
constexpr uint8_t compileDate[] = __DATE__;  // Seed B for the MAC address generation

class macAddressHelper {
   private:
    uint8_t _mac[6];  // MAC address

   public:
    macAddressHelper();
    /**
     * @brief Construct a new mac Address Helper object
     *
     */
    macAddressHelper(uint8_t hostAddressOctet);

    ~macAddressHelper();

    /**
     * @brief Get the Mac address and store it in the buffer, non-architectural specific
     *
     * @param macBuffer , uint8_t[6] buffer to store the MAC address
     * @return true, if the MAC address is successfully read
     * @return false, if the MAC address is not successfully read
     */
    bool getMac(uint8_t* macBuffer);
};
}  // namespace macGen

#endif