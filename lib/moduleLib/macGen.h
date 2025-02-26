#ifndef MACGEN_H
#define MACGEN_H

#ifdef __AVR__
#include <Arduino.h>
#include <EEPROM.h>
#else
//  For non-AVR systems
#endif

#include <stdint.h>

// Set the default debug mode for the macGen
#ifndef DEBUG
#define DEBUG false
#endif

namespace macGenConstants {
constexpr uint8_t AVR_MAC_MEM_ADDRESS[6] = {10, 11, 12,
                                            13, 14, 15};  // MAC address locations in EEPROM
}  // namespace macGenConstants

namespace macGen {

class macAddressHelper {
   private:
    uint8_t _mac[6];  // MAC address

#if defined(__AVR__)
    /**
     * @brief Get the Mac From E E P R O M object
     *
     * @param macBuffer , uint8_t[6] buffer to store the MAC address
     * @return true, if the MAC address is successfully read from EEPROM
     * @return false, if the MAC address is not successfully read from EEPROM
     */
    bool _getMacFromEEPROM(uint8_t* macBuffer);

    /**
     * @brief Set the Mac In E E P R O M object
     *
     * @param newMac , uint8_t[6] buffer to read the new MAC address
     * @return true, if the MAC address is successfully written to EEPROM
     * @return false, if the MAC address is not successfully written to EEPROM
     */
    bool _updateMacInEEPROM(uint8_t* newMac);
#else
// For non-AVR systems
#error "Architecture not yet supported"
#endif

    /**
     * @brief Generate a MAC address
     *
     * @param macBuffer , uint8_t[6] buffer to store the MAC address
     */
    void _generateMac(uint8_t* macBuffer);

   public:
    /**
     * @brief Construct a new mac Address Helper object
     *
     */
    macAddressHelper();

    /**
     * @brief Get the Mac address and store it in the buffer, non-architectural specific
     *
     * @param macBuffer , uint8_t[6] buffer to store the MAC address
     * @return true, if the MAC address is successfully read
     * @return false, if the MAC address is not successfully read
     */
    bool getMac(uint8_t* macBuffer);

    /**
     * @brief Set the MAC address to a new MAC address, non-architectural specific
     *
     * @param newMac , uint8_t[6] buffer to read the new MAC address
     * @return true, if the MAC address is successfully overwritten
     * @return false, if the MAC address is not successfully overwritten
     */
    bool overwriteMac(uint8_t* newMac);
};
}  // namespace macGen

#endif