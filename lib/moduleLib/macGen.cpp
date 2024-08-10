#include "macGen.h"

using namespace macGen;

// -- Private methods -- //

#if defined(__AVR__)
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
#else
// For non-AVR systems

#endif

// -- Public methods -- //

macAddressHelper::macAddressHelper() {
#if defined(__AVR__)
    // Get MAC address from EEPROM
    if (!getMacFromEEPROM(this->mac) || this->mac[0] == 0 || this->mac[0] == 0xff) {
        // If no MAC address in EEPROM, generate one
#if DEBUG
        Serial.println(F("No MAC address in EEPROM, generating one"));
#endif

        generateMac(this->mac);
        updateMacInEEPROM(this->mac);
    }
#else
// For non-AVR systems
#endif
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
#if defined(__AVR__)
#if DEBUG
    Serial.println(F("Updating MAC address in EEPROM"));
#endif
    return updateMacInEEPROM(this->mac);
#else
    // For non-AVR systems
    return true;
#endif
}

void macAddressHelper::generateMac(uint8_t* macBuffer) {
// Generate a MAC address
#if defined(__AVR__)
#if DEBUG
    Serial.println(F("Generating MAC address"));
#endif
    for (int i = 0; i < 6; i++) {
        macBuffer[i] = random(0, 255);
#if DEBUG
        Serial.print(macBuffer[i], HEX);
        Serial.print(F(":"));
#endif
    }
#else
// For non-AVR systems
#endif
}