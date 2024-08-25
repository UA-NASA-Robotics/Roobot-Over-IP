#include "blacklistManager.h"

BlacklistManager::BlacklistManager() {
    for (int i = 0; i < 255; i++) {
        blackList[i] = false;
    }
}

void BlacklistManager::addBlacklist(uint8_t octet) { blackList[octet] = true; }

void BlacklistManager::removeBlacklist(uint8_t octet) { blackList[octet] = false; }

void BlacklistManager::exportBlacklist(uint8_t* buffer, uint8_t maxExportLength) {
    uint8_t index = 0;
    for (int i = 0; i < maxExportLength; i++) {
        if (blackList[i]) {
            buffer[index] = i;
            index++;
        }
    }
}

bool BlacklistManager::verifyOctet(uint8_t octet) { return blackList[octet]; }