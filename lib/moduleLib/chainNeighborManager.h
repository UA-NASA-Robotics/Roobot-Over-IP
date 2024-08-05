#ifndef chainManager_H
#define chainManager_H

#if defined(__AVR__)
#include <Arduino.h>
#else
// For non-AVR systems
#endif

#include <Ethernet.h>  // Ethernet library, we need this to send packets in discoverChain and chainForward
#include <stdint.h>

#include "../Packet.h"
#include "statusManager.h"

// Set the default debug mode for the chainNeighborManager
#ifndef DEBUG
#define DEBUG false
#endif

namespace chainManagerConstants {

constexpr uint8_t CHAINTIMEOUT =
    50;  // Response timeout for chain neighbor discovery, in milliseconds
constexpr uint8_t CHAINCHECKINTERVAL = 50;  // Number of cycles between chain checks

constexpr uint16_t NULLOCTET = 300;  // Null octet for chain discovery

}  // namespace chainManagerConstants
namespace chainNeighborManager {
class chainNeighborManager {
   private:
    bool chainNeighborConnected;  // Chain neighbor connected status
    bool chainOperational;        // Chain operational status

    uint8_t moduleType;  // Module type (see moduleTypesConstants in ModuleCodec.h)

    uint8_t NetworkAddress[4];  // Network address of system
    uint8_t hostOctet;          // Host octet of the module
    uint8_t neighborOctet;      // Chain Neighbor octet of the module

    statusManager::statusManager statusManager;  // Helper class to callback to

    EthernetUDP sysAdmin;  // UDP object to send sysAdmin packets

    uint8_t* generalBuffer;  // General buffer for use in the class (used for packet data)

    uint8_t timeUntilChainCheck;  // Cycles until the entire chain is checked again
    uint8_t lastOctetChecked;     // Last octet checked in the chain

    bool doDiscovery;  // Whether the module should do discovery (Activated by ISR)

    bool pingModule(uint8_t clientAddressOctet);  // Ping the chain neighbor to make sure it is
                                                  // still there, True if the ping is successful

    int16_t pingChain();  // Ping the entire chain to make sure it is still there, returns the
                          // number of modules in the chain, -1 if the chain is broken

    uint16_t pingRangeMinima(uint8_t minimumOctet,
                             uint8_t maximumOctet);  // Ping a range of
                                                     // octets (0 wrap works), returns the minima
                                                     // octet or NULLOCTET if no module is found

   public:
    chainNeighborManager();  // Default constructor (THIS CANNOT BE USED, IT IS HERE FOR OBJECT
                             // FIELDS. DO NOT USE)

    chainNeighborManager(uint16_t moduleType, uint8_t* networkAddress, uint8_t hostOctet,
                         statusManager::statusManager statusManager, EthernetUDP sysAdmin,
                         uint8_t* generalBuffer);  // Constructor

    ~chainNeighborManager();  // Destructor

    bool getChainNeighborConnected();  // Get the chain neighbor connected status
    bool getChainOperational();        // Get the chain operational status

    uint8_t getChainNeighborOctet();  // Get the chain neighbor octet

    void notifyDoDiscovery();  // Whether the module should notify the ISR to do discovery

    void discoverChain();  // Give the module a chance to discover its chain neighbors, and manage
                           // the chain (An update function)

    bool chainForward(
        ROIPackets::sysAdminPacket
            packet);  // Forward a packet to the next module in the chain. sysAdminHandler will call
                      // this function, and can generate a reply packet that Must NOT be forwarded.
};

}  // namespace chainNeighborManager

#endif