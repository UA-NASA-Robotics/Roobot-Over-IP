#ifndef statusManager_H
#define statusManager_H

#include <stdint.h>

namespace statusManager {
class statusManager {
   private:
    uint8_t systemStatus;  // The status of the system

   public:
    statusManager();  // Constructor

    uint8_t getSystemStatus();  // Get the status of the system
    bool getOperable();         // Get whether the system is operable

    void ConfiguredCallback();  // Callback for when the system is configured, call when the module
    // receives any configuration

    void ErrorCallback(bool inoperable);  // Callback for when the system encounters an error, call
    // when the module encounters an error.

    void chainNeighborCallback(
        bool neighborAcquired,
        bool chainFunctional);  // Callback from the chain manager to indicate if a neighbor has
    // been acquired and if the chain is functional
};
};  // namespace statusManager

#endif
