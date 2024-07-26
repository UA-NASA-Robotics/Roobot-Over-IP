#ifndef chainManager_H
#define chainManager_H

#include <stdint.h>

#include "../../lib/Packet.h"
#include "Arduino.h"
#include "statusManager.h"

namespace chainNeighborManager {
class chainNeighborManager {
   private:
    uint8_t moduleType;  // Module type (see moduleTypesConstants in ModuleCodec.h)

    uint8_t NetworkAddress[4];  // Network address of system
    uint8_t hostOctet;          // Host octet of the module
    uint8_t NeighborOctet;      // Chain Neighbor octet of the module

    statusManager::statusManager statusManager;  // Helper class to callback to

   public:
    chainNeighborManager(uint8_t moduleType, uint8_t* networkAddress, uint8_t hostOctet,
                         uint8_t NeighborOctet,
                         statusManager::statusManager statusManager);  // Constructor

    ~chainNeighborManager();  // Destructor

    void DiscoverChain();  // Give the module a chance to discover its chain neighbors, and manage
                           // the chain (An update function)

    bool ChainForward(
        ROIPackets::sysAdminPacket
            packet);  // Forward a packet to the next module in the chain. sysAdminHandler will call
                      // this function, and can generate a reply packet that Must NOT be forwarded.
};

}  // namespace chainNeighborManager