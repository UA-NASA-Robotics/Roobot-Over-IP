#include "blacklistManager.h"

BlacklistManager::BlacklistManager() {
    for (int i = 0; i < 255; i++) {
        _blackList[i] = false;
    }
}

void BlacklistManager::addBlacklist(uint8_t octet) {
    _blackList[octet] = true;
#if DEBUG && defined(__AVR__)
    Serial.print(F("Blacklisted octet: "));
    Serial.println(octet);
#endif
}

void BlacklistManager::removeBlacklist(uint8_t octet) {
    _blackList[octet] = false;
#if DEBUG && defined(__AVR__)
    Serial.print(F("Removed octet from blacklist: "));
    Serial.println(octet);
#endif
}

void BlacklistManager::exportBlacklist(uint8_t* buffer, uint8_t maxExportLength) {
    uint8_t index = 0;
    for (int i = 0; i < maxExportLength; i++) {
        if (_blackList[i]) {
            buffer[index] = i;
            index++;
        }
    }

#if DEBUG && defined(__AVR__)
    Serial.println(F("Exported blacklist"));
#endif
}

bool BlacklistManager::verifyOctet(uint8_t octet) {
    return _blackList[octet];
#if DEBUG && defined(__AVR__)
    Serial.print(F("Verified octet: "));
    Serial.println(octet);
#endif
}