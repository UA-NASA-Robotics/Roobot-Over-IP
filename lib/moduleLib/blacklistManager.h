#ifndef BLACKLISTMANAGER_H
#define BLACKLISTMANAGER_H

#if defined(__AVR__)
#include <Arduino.h>
#else
// For non-AVR systems
#endif

#include <stdint.h>

// Set the default debug mode for the blacklistManager
#ifndef DEBUG
#define DEBUG false
#endif

class BlacklistManager {
   private:
    bool blackList[255];  // Official list of octets that are blacklisted, 1 if blacklisted, 0 if
                          // not

   public:
    BlacklistManager();  // Constructor for the BlacklistManager class

    void addBlacklist(uint8_t octet);  // Add an octet to the blacklist

    void removeBlacklist(uint8_t octet);  // Remove an octet from the blacklist

    void exportBlacklist(uint8_t* buffer,
                         uint8_t maxExportLength);  // Export the blacklist to a buffer

    bool verifyOctet(uint8_t octet);  // Verify if an octet is blacklisted
};

#endif  // BLACKLISTMANAGER_H