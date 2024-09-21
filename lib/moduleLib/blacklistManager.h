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
    /**
     * @brief Construct a new Blacklist Manager object
     *
     */
    BlacklistManager();

    /**
     * @brief  Add an octet to the blacklist
     *
     * @param octet  The host address octet to add to the blacklist
     */
    void addBlacklist(uint8_t octet);

    /**
     * @brief  Remove an octet from the blacklist
     *
     * @param octet  The host address octet to remove from the blacklist
     */
    void removeBlacklist(uint8_t octet);

    /**
     * @brief  Export the blacklist to a buffer
     *
     * @param buffer  The buffer to export the blacklist to
     * @param maxExportLength  The maximum length of the buffer
     */
    void exportBlacklist(uint8_t* buffer, uint8_t maxExportLength);

    /**
     * @brief  Verify if an octet is blacklisted
     *
     * @param octet  The host address octet to verify
     * @return true  If the octet is blacklisted
     * @return false   If the octet is not blacklisted
     */
    bool verifyOctet(uint8_t octet);
};

#endif  // BLACKLISTMANAGER_H