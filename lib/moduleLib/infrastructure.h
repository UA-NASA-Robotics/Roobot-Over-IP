#ifndef INFRA_H
#define INFRA_H

// Define the default debug mode for the ROI module
#ifndef DEBUG
#define DEBUG false
#endif

#if defined(__AVR__)
#include <Arduino.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#else
// For non-AVR systems
#endif

#include "../ModuleCodec.h"
#include "../Packet.h"
#include "../floatCast.h"
#include "IPContainer.h"
#include "blacklistManager.h"
#include "chainNeighborManager.h"
#include "macGen.h"
#include "octetSelector.h"
#include "statusManager.h"
#include "sysAdminHandler.h"

namespace InfrastructureConstants {
constexpr uint8_t NETWORK_ADDRESS1 = 192;  // Define the first part of the network address
constexpr uint8_t NETWORK_ADDRESS2 = 168;  // Define the second part of the network address
constexpr uint8_t NETWORK_ADDRESS3 = 2;    // Define the third part of the network address

constexpr uint8_t RETRANSMISSION_COUNT = 1;   // Define the retransmission count
constexpr uint8_t RETRANSMISSION_TIME = 100;  // Define the retransmission time (unit is 100us) 100
                                              // = 10ms

constexpr bool IGNORE_BLACKLIST = false;  // Define if the blacklist should be ignored
constexpr bool IGNORE_CHECKSUM_FAILURE =
    true;  // Define if checksum failure should be ignored, allows comprehension of potential
           // corrupted packets
}  // namespace InfrastructureConstants

class ModuleInfrastructure {
   private:
    uint8_t WIZ5500_CS_PIN;  // Chip select pin for WIZ5500 module
    uint8_t moduleType;      // Module type (see moduleTypesConstants in ModuleCodec.h)

    macGen::macAddressHelper macHelper;
    uint8_t mac[6];

    OctetSelectorRev1* selector;  // Create an octet selector instance

    IPContainer moduleIPContainer;  // Define network address in platformio.ini

    BlacklistManager moduleBlacklistManager;  // Create a blacklist manager instance

    // Create a UDP instances for each type of packet on the ROI module
    EthernetUDP General;
    EthernetUDP Interrupt;
    EthernetUDP SysAdmin;

    chainNeighborManager::chainNeighborManager
        moduleChainManager;  // Create a chainNeighborManager instance

    sysAdminHandler::sysAdminHandler moduleSysAdminHandler;  // Create a sysAdminHandler instance

    ROIPackets::Packet (*handleGeneralPacket)(ROIPackets::Packet);  // Function pointer to handle
    // general packets

#if defined(__AVR__)
    /**
     * @brief Set the Hardware Interrupt Timer to call chain manager to do discovery
     *
     */
    void setHardwareInterruptTimer();
#endif

   public:
    statusManager::statusManager moduleStatusManager;  // Create a status manager instance (manages
                                                       // the status of the ROI module)

    uint8_t generalBuffer[ROIConstants::ROIMAXPACKETSIZE];  // Buffer for packet import and export

    void (*resetFunction)(void) = 0;  // declare reset function @ address 0

    /**
     * @brief Construct a new Module Infrastructure object
     *
     * @param W5500_CS_Pin , the chip select pin for the W5500 module
     * @param octetSelectorRev , the revision of the octet selector
     */
    ModuleInfrastructure(uint8_t W5500_CS_Pin = 10, uint8_t octetSelectorRev = 2,
                         moduleTypesConstants::moduleTypeConstant moduleType = 0,
                         ROIPackets::Packet (*handler)(ROIPackets::Packet) = (nullptr));

    /**
     * @brief Creates all the necessary instances and initializes the module infrastructure
     *
     */
    void init();

    /**
     * @brief Passes the interrupt notification to the chain manager
     *
     */
    void interruptNotification();

    /**
     * @brief Void loop function for the module infrastructure to process packets
     *
     */
    void tick();
};

#endif