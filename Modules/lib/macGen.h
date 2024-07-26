#ifndef MACGEN_H
#define MACGEN_H

#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>

namespace macGenConstants {
    const uint8_t macLocations[6] = { 10, 11, 12, 13, 14, 15 };  // MAC address locations in EEPROM
}  // namespace macGenConstants

namespace macGen {

    class macAddressHelper {
    private:
        uint8_t mac[6];  // MAC address

        bool getMacFromEEPROM(uint8_t* macBuffer);

        bool updateMacInEEPROM(uint8_t* newMac);

        void generateMac(uint8_t* macBuffer);

    public:
        macAddressHelper();

        bool getMac(uint8_t* macBuffer);

        bool overwriteMac(uint8_t* newMac);
    };
}  // namespace macGen

#endif