#include "macGen.h"

using namespace macGen;

// -- Private methods -- //

#if defined(__AVR__)
bool macAddressHelper::_getMacFromEEPROM(uint8_t* macBuffer) {
    for (int i = 0; i < 6; i++) {
        macBuffer[i] = EEPROM.read(macGenConstants::AVR_MAC_MEM_ADDRESS[i]);
    }
    return true;
}

bool macAddressHelper::_updateMacInEEPROM(uint8_t* newMac) {
    for (int i = 0; i < 6; i++) {
        EEPROM.write(macGenConstants::AVR_MAC_MEM_ADDRESS[i], newMac[i]);
    }
    return true;
}
#else
// For non-AVR systems
#error "Architecture not yet supported"

#endif

void macAddressHelper::_generateMac(uint8_t* macBuffer) {
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
#error "Architecture not yet supported"
#endif
}

// -- Public methods -- //

macAddressHelper::macAddressHelper() {
#if defined(__AVR__)
    // Get MAC address from EEPROM
    if (!_getMacFromEEPROM(this->_mac) || this->_mac[0] == 0 || this->_mac[0] == 0xff) {
        // If no MAC address in EEPROM, generate one
#if DEBUG
        Serial.println(F("No MAC address in EEPROM, generating one"));
#endif

        _generateMac(this->_mac);
        _updateMacInEEPROM(this->_mac);
    }
#else
// For non-AVR systems
#error "Architecture not yet supported"
#endif
}

bool macAddressHelper::getMac(uint8_t* macBuffer) {
    for (int i = 0; i < 6; i++) {
        macBuffer[i] = this->_mac[i];
    }
    return true;
}

bool macAddressHelper::overwriteMac(uint8_t* newMac) {
    for (int i = 0; i < 6; i++) {
        this->_mac[i] = newMac[i];
    }
#if defined(__AVR__)
#if DEBUG
    Serial.println(F("Updating MAC address in EEPROM"));
#endif
    return _updateMacInEEPROM(this->_mac);
#else
// For non-AVR systems
#error "Architecture not yet supported"
#endif
}