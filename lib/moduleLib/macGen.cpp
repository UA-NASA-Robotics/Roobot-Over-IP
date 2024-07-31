#include "macGen.h"

using namespace macGen;

macAddressHelper::macAddressHelper() {
    // Get MAC address from EEPROM
    if (!getMacFromEEPROM(this->mac) || this->mac[0] == 0 || this->mac[0] == 0xff) {
        // If no MAC address in EEPROM, generate one
        generateMac(this->mac);
        updateMacInEEPROM(this->mac);
    }
}

bool macAddressHelper::getMac(uint8_t* macBuffer) {
    for (int i = 0; i < 6; i++) {
        macBuffer[i] = this->mac[i];
    }
    return true;
}

bool macAddressHelper::overwriteMac(uint8_t* newMac) {
    for (int i = 0; i < 6; i++) {
        this->mac[i] = newMac[i];
    }
    return updateMacInEEPROM(this->mac);
}

bool macAddressHelper::getMacFromEEPROM(uint8_t* macBuffer) {
    for (int i = 0; i < 6; i++) {
        macBuffer[i] = EEPROM.read(macGenConstants::macLocations[i]);
    }
    return true;
}

bool macAddressHelper::updateMacInEEPROM(uint8_t* newMac) {
    for (int i = 0; i < 6; i++) {
        EEPROM.write(macGenConstants::macLocations[i], newMac[i]);
    }
    return true;
}

void macAddressHelper::generateMac(uint8_t* macBuffer) {
    // Generate a MAC address
    for (int i = 0; i < 6; i++) {
        macBuffer[i] = random(0, 255);
    }
}