#ifndef IPWRAPPER_H
#define IPWRAPPER_H

#include <Ethernet2.h>  // Ethernet library, we need this to send packets in discoverChain and chainForward
#include <EthernetUdp2.h>  // Ethernet UDP library, we need this to send packets in discoverChain and chainForward
#include <stdint.h>

#include "octetSelector.h"

class IPWrapper {
   private:
    OctetSelectorRev1* octetSelector;  // Octet selector object

   public:
    /**
     * @brief Construct a new IPWrapper object
     *
     */
    IPWrapper(OctetSelectorRev1* selector, uint8_t networkHighByte, uint8_t networkHighMidByte,
              uint8_t networkLowMidByte);  // Constructor

    /**
     * @brief Inits the IPWrapper object and reads the IP from an octet selector
     *
     */
    void init();

    uint8_t addressArray[4];   // Network address of system
    IPAddress networkAddress;  // Network address of system
};

#endif