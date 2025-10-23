#include "blacklistManager.h"

BlacklistManager::BlacklistManager() {
    for (int i = 0; i < 255; i++) {
        _blackList[i] = false;
    }
}

void BlacklistManager::addBlacklist(uint8_t octet) {
    _blackList[octet] = true;
    __debug_info_val("Added octet to blacklist: ", octet);
}

void BlacklistManager::removeBlacklist(uint8_t octet) {
    _blackList[octet] = false;
    __debug_info_val("Removed octet from blacklist: ", octet);
}

void BlacklistManager::exportBlacklist(uint8_t* buffer, uint8_t maxExportLength) {
    uint8_t index = 0;
    for (int i = 0; i < maxExportLength; i++) {
        if (_blackList[i]) {
            buffer[index] = i;
            index++;
        }
    }

    __debug_info("Exported blacklist");
}

bool BlacklistManager::verifyOctet(uint8_t octet) {
    if (_blackList[octet]) {
        __debug_event_val("Blacklisted octet occurred: ", octet);
        return true;
    }
    __debug_info_val("Verified octet: ", octet);
    return false;
}