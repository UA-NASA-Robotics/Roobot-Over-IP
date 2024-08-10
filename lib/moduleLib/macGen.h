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
const uint8_t macLocations[6] = {10, 11, 12, 13, 14, 15};  // MAC address locations in EEPROM
}  // namespace macGenConstants

namespace macGen {

class macAddressHelper {
   private:
    uint8_t mac[6];  // MAC address

#if defined(__AVR__)
    bool getMacFromEEPROM(uint8_t* macBuffer);

    bool updateMacInEEPROM(uint8_t* newMac);
#else
// For non-AVR systems
#endif

    void generateMac(uint8_t* macBuffer);

   public:
    macAddressHelper();

    bool getMac(uint8_t* macBuffer);

    bool overwriteMac(uint8_t* newMac);
};
}  // namespace macGen

#endif