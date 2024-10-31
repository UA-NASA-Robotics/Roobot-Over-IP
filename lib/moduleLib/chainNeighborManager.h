#ifndef chainManager_H
#define chainManager_H

#if defined(__AVR__)
#include <Arduino.h>
#else
// For non-AVR systems
#endif

#include <Ethernet2.h>  // Ethernet library, we need this to send packets in discoverChain and chainForward
#include <EthernetUdp2.h>  // Ethernet UDP library, we need this to send packets in discoverChain and chainForward
#include <stdint.h>

#include "../Packet.h"
#include "IPWrapper.h"
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

    IPContainer ipWrapper;  // IPContainer object for the module
    uint8_t neighborOctet;  // Chain Neighbor octet of the module

    statusManager::statusManager& statusManager;  // Helper class to callback to

    EthernetUDP& sysAdmin;  // UDP object to send sysAdmin packets

    uint8_t* generalBuffer;  // General buffer for use in the class (used for packet data)

    uint8_t timeUntilChainCheck;  // Cycles until the entire chain is checked again
    uint8_t lastOctetChecked;     // Last octet checked in the chain

    bool doDiscovery;  // Whether the module should do discovery (Activated by ISR)

    /**
     * @brief  Ping any module by its octet
     *
     * @param clientAddressOctet
     * @return true  If the ping is successful
     * @return false  If the ping is unsuccessful
     */
    bool pingModule(uint8_t clientAddressOctet);

    /**
     * @brief  Ping the entire chain
     *
     * @return int16_t > 0 The number of modules in the chain
     * @return int16_t == -1 if the chain is broken
     */
    int16_t pingChain();

    /**
     * @brief  Ping a range of octets
     *
     * @param minimumOctet , inclusive test minimum octet
     * @param maximumOctet , inclusive test maximum octet
     * @return uint16_t  The minima octet or chainNeighborConstants::NULLOCTET if no module is found
     */
    uint16_t pingRangeMinima(uint8_t minimumOctet, uint8_t maximumOctet);

   public:
    /**
     * @brief Construct a new chain Neighbor Manager object
     *
     * @param moduleType , moduleTypesConstants::*type*
     * @param networkAddress , uint8_t[4] network address
     * @param hostOctet , host octet networkAddress[4] (to be deprecated)
     * @param statusManager , already initialized statusManager for the module
     * @param sysAdmin , EthernetUDP object for sysAdmin packets
     * @param generalBuffer , uint8_t* general buffer for use in the class
     */
    chainNeighborManager(uint16_t moduleType, IPContainer& IPContainer,
                         statusManager::statusManager& statusManager, EthernetUDP& sysAdmin,
                         uint8_t* generalBuffer);  // Constructor

    /**
     * @brief Destroy the chain Neighbor Manager object
     *
     */
    ~chainNeighborManager();  // Destructor

    /**
     * @brief Get if the chain neighbor is connected
     *
     * @return true, if the chain neighbor is connected
     * @return false, if the chain neighbor is not connected
     */
    bool getChainNeighborConnected();

    /**
     * @brief Get if the chain is operational
     *
     * @return true, if the chain is operational
     * @return false, if the chain is not operational
     */
    bool getChainOperational();

    /**
     * @brief Get the chain neighbor host octet
     *
     * @return uint8_t, the host octet
     */
    uint8_t getChainNeighborOctet();

    /**
     * @brief Function called by a timer ISR to notify the module to do discovery on the next void
     * loop cycle
     *
     */
    void notifyDoDiscovery();

    /**
     * @brief  Void loop worker function for the chainNeighborManager to manage the chain. Could be
     * threaded on a multi-core system
     *
     */
    void discoverChain();

    /**
     * @brief  Forward a packet to the next module in the chain. sysAdminHandler will call this
     * function, and can generate a reply packet that Must NOT be forwarded.
     *
     * @param packet
     * @return true, if the packet is forwarded successfully
     * @return false, if the packet is not forwarded successfully
     */
    bool chainForward(ROIPackets::sysAdminPacket packet);
};

}  // namespace chainNeighborManager

#endif