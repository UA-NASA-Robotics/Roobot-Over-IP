#ifndef chainManager_H
#define chainManager_H

#include <Ethernet.h>  // Ethernet library, we need this to send packets in discoverChain and chainForward
#include <stdint.h>

#include "../../lib/Packet.h"
#include "Arduino.h"
#include "statusManager.h"

namespace chainManagerConstants {

const uint8_t CHAINTIMEOUT = 300;  // Response timeout for chain neighbor discovery, in milliseconds

}  // namespace chainManagerConstants
namespace chainNeighborManager {
class chainNeighborManager {
   private:
    bool chainNeighborConnected;  // Chain neighbor connected status
    bool chainOperational;        // Chain operational status

    uint8_t moduleType;  // Module type (see moduleTypesConstants in ModuleCodec.h)

    uint8_t NetworkAddress[4];  // Network address of system
    uint8_t hostOctet;          // Host octet of the module
    uint8_t NeighborOctet;      // Chain Neighbor octet of the module

    statusManager::statusManager statusManager;  // Helper class to callback to

    EthernetUDP sysAdmin;  // UDP object to send sysAdmin packets

    uint8_t* generalBuffer;  // General buffer for use in the class (used for packet data)

    uint8_t timeUntilChainCheck;  // Cycles until the entire chain is checked again
    uint8_t lastOctetChecked;     // Last octet checked in the chain

    bool pingModule(uint8_t clientAddressOctet);  // Ping the chain neighbor to make sure it is
                                                  // still there, True if the ping is successful

   public:
    chainNeighborManager(uint8_t moduleType, uint8_t* networkAddress, uint8_t hostOctet,
                         uint8_t NeighborOctet, statusManager::statusManager statusManager,
                         EthernetUDP sysAdmin, uint8_t* generalBuffer);  // Constructor

    ~chainNeighborManager();  // Destructor

    bool getChainNeighborConnected();  // Get the chain neighbor connected status
    bool getChainOperational();        // Get the chain operational status

    uint8_t getChainNeighborOctet();  // Get the chain neighbor octet

    void discoverChain();  // Give the module a chance to discover its chain neighbors, and manage
                           // the chain (An update function)

    bool chainForward(
        ROIPackets::sysAdminPacket
            packet);  // Forward a packet to the next module in the chain. sysAdminHandler will call
                      // this function, and can generate a reply packet that Must NOT be forwarded.
};

}  // namespace chainNeighborManager