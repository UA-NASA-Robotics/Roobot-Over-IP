#ifndef statusManager_H
#define statusManager_H

#include <stdint.h>

#include "../../lib/ModuleCodec.h"

namespace statusManager {
class statusManager {
   private:
    bool initialized;       // Whether the system is initialized
    bool configured;        // Whether the system is configured
    bool chainFunctional;   // Whether the chain is functional
    bool neighborAcquired;  // Whether a neighbor has been acquired
    bool hasError;          // Whether the system has an error
    bool errorInoperable;   // Whether the error is inoperable

   public:
    statusManager();  // Constructor

    ~statusManager();  // Destructor

    uint8_t getSystemStatus();  // Get the status of the system
    bool getOperable();         // Get whether the system is operable

    void
    initializedCallback();  // Callback for when the system is initialized, call when the module

    void configuredCallback();  // Callback for when the system is configured, call when the module
    // receives any configuration

    void errorCallback(bool inoperable);  // Callback for when the system encounters an error, call
    // when the module encounters an error.

    void clearError();  // Clear the error

    void chainNeighborCallback(
        bool neighborAcquired,
        bool chainFunctional);  // Callback from the chain manager to indicate if a neighbor has
    // been acquired and if the chain is functional
};
};  // namespace statusManager

#endif
