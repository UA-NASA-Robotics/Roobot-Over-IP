#ifndef sysAdminHandler_H
#define sysAdminHandler_H

#include "../ModuleCodec.h"
#include "../Packet.h"
#include "chainNeighborManager.h"
#include "macGen.h"
#include "statusManager.h"
#include "supplyVoltage.h"

#ifndef DEBUG
#define DEBUG false
#endif

namespace sysAdminHandler {

class sysAdminHandler {
   private:
    uint8_t moduleType;  // Module type (see moduleTypesConstants in ModuleCodec.h)

    uint8_t mac[6];  // MAC address cache

    statusManager::statusManager& statusManager;  // Helper class to get the status of the system

    chainNeighborManager::chainNeighborManager& chainManager;  // Helper class to manage the chain

    uint8_t* generalBuffer;  // General buffer for use in the class (used for packet data)

   public:
    sysAdminHandler(uint16_t moduleType, statusManager::statusManager& statusManager,
                    chainNeighborManager::chainNeighborManager& chainManager,
                    uint8_t* generalBuffer,
                    uint8_t* MACAddress);  // Constructor

    ~sysAdminHandler();  // Destructor

    ROIPackets::sysAdminPacket handleSysAdminPacket(
        ROIPackets::sysAdminPacket packet);  // Function to handle sysAdmin packets
};

}  // namespace sysAdminHandler

#endif