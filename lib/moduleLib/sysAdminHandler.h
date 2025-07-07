#ifndef sysAdminHandler_H
#define sysAdminHandler_H

#include "../Packet.h"
#include "../UDP-API/moduleType.h"
#include "../UDP-API/packetTypes.h"
#include "../UDP-API/sysAdmin.h"
#include "blacklistManager.h"
#include "chainNeighborManager.h"
#include "debug.h"
#include "statusManager.h"
#include "supplyVoltage.h"

namespace sysAdminHandler {

constexpr uint8_t compileTime[] = __TIME__;  // Seed A for the MAC address generation
constexpr uint8_t compileDate[] = __DATE__;  // Seed B for the MAC address generation

class sysAdminHandler {
   private:
    uint8_t _moduleType;  // Module type (see moduleTypesConstants in ModuleCodec.h)

    uint8_t _mac[6];  // MAC address cache

    statusManager::statusManager& _statusManager;  // Helper class to get the status of the system

    chainNeighborManager::chainNeighborManager& _chainManager;  // Helper class to manage the chain

    BlacklistManager& _blacklistManager;  // Helper class to manage the blacklist

    uint8_t* _generalBuffer;  // General buffer for use in the class (used for packet data)

   public:
    /**
     * @brief Construct a new sys Admin Handler object
     *
     * @param moduleType , uint16_t module type see moduleTypesConstants in ModuleCodec.h
     * @param statusManager , statusManager object of the module (pre-initialized)
     * @param chainManager , chainNeighborManager object of the module (pre-initialized)
     * @param blacklistManager , BlacklistManager object of the module (pre-initialized)
     * @param generalBuffer , uint8_t* general buffer for use in the class (Can be shared with other
     * classes on single threaded systems)
     */
    sysAdminHandler(uint16_t moduleType, statusManager::statusManager& statusManager,
                    chainNeighborManager::chainNeighborManager& chainManager,
                    BlacklistManager& blacklistManager, uint8_t* generalBuffer);

    /**
     * @brief Destroy the sys Admin Handler object
     *
     */
    ~sysAdminHandler();

    /**
     * @brief Set the MAC address of the module in the sysAdminHandler (Called in void setup() after
     * mac EEPROM is read)
     *
     * @param mac , uint8_t[6] buffer containing the MAC address
     */
    void setMAC(uint8_t* mac);

    /**
     * @brief Handle a sysAdmin packet, and return a sysAdmin packet response (Forwards to the
     * appropriate handler if needed)
     *
     * @param packet , ROIPackets::sysAdminPacket sysAdmin packet to handle
     * @return ROIPackets::sysAdminPacket, sysAdmin packet response
     */
    ROIPackets::sysAdminPacket handleSysAdminPacket(ROIPackets::sysAdminPacket packet);
};

}  // namespace sysAdminHandler

#endif