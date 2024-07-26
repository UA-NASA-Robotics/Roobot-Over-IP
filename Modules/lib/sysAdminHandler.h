#ifndef sysAdminHandler_H
#define sysAdminHandler_H

#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "macGen.h"
#include "statusManager.h"
#include "supplyVoltage.h"

namespace sysAdminHandler {

    class sysAdminHandler {
    private:
        uint8_t moduleType;  // Module type (see moduleTypesConstants in ModuleCodec.h)

        macGen::macAddressHelper macHelper;  // Helper class to get the MAC address
        uint8_t mac[6];                      // MAC address cache

        statusManager::statusManager statusManager;  // Helper class to get the status of the system

    public:
        sysAdminHandler(uint8_t moduleType, statusManager::statusManager manager);  // Constructor

        ~sysAdminHandler();  // Destructor

        ROIPackets::sysAdminPacket handleSysAdminPacket(
            ROIPackets::sysAdminPacket packet);  // Function to handle sysAdmin packets
    };

}  // namespace sysAdminHandler

#endif