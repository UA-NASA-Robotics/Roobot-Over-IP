#ifndef IPWRAPPER_H
#define IPWRAPPER_H

#include <Ethernet2.h>  // Ethernet library, we need this to send packets in discoverChain and chainForward
#include <EthernetUdp2.h>  // Ethernet UDP library, we need this to send packets in discoverChain and chainForward
#include <stdint.h>

#include "octetSelector.h"

struct IPContainer {
    uint8_t addressArray[4];   // Network address of system
    IPAddress networkAddress;  // Network address of system

    /**
     * @brief Construct a new IPContainer object
     *
     */
    IPContainer(uint8_t networkHighByte, uint8_t networkHighMidByte, uint8_t networkLowMidByte,
                uint8_t networkLowByte);  // Constructor

    /**
     * @brief Inits the IPContainer object and reads the IP from an octet selector
     *
     */
    void updateIP(uint8_t networkHighByte, uint8_t networkHighMidByte, uint8_t networkLowMidByte,
                  uint8_t networkLowByte);  // Update the IP address
    void updateIP(uint8_t hostAddress);     // Update the IP address
};

#endif