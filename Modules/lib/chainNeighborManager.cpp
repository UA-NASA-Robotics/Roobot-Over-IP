#include "chainNeighborManager.h"

// --- PRIVATE FUNCTIONS --- //

bool pingModule(uint8_t clientAddressOctet) {
    ROIPackets::sysAdminPacket pingPacket;  // Create a sysAdminPacket object that will be used
                                            // to ping the module
    IPAddress moduleIP(NetworkAddress[0], NetworkAddress[1], NetworkAddress[2],
                       clientAddressOctet);  // Create an IPAddress object for the module
    pingPacket.setHostAddressOctet(hostOctet);
    pingPacket.setClientAddressOctet(clientAddressOctet);
    pingPacket.setAdminMetaData(sysAdminConstants::NOCHAINMETA);
    pingPacket.setActionCode(sysAdminConstants::PING);

    pingPacket.exportPacket(
        generalBuffer,
        ROIConstants::ROIMAXPACKETSIZE);  // Export the packet to the general buffer

    sysAdmin.beginPacket(moduleIP,
                         ROIConstants::ROISYSADMINPORT);  // Send the ping packet to the module
    sysAdmin.write(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);
    if (!sysAdmin.endPacket();) {
        // If the packet fails to send, then the chain neighbor is no longer connected.
        return false;
    }

    // now wait for a response from the chain neighbor
    long startTime = millis();
    int packetReceived = 0;
    while (millis() - startTime < chainManagerConstants::CHAINTIMEOUT && !packetReceived) {
        packetReceived = sysAdmin.parsePacket();  // Check if a packet is received from the chain
    }

    if (packetReceived) {  // A packet was received from module, check it is coherent.
        sysAdmin.read(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);

        ROIPackets::sysAdminPacket responsePacket;
        if (!responsePacket.importPacket(generalBuffer, ROIConstants::ROIMAXPACKETSIZE)) {
            return false;  // If the packet is not coherent, then the ping failed.
        };
        return responsePacket.getActionCode() ==
               sysAdminConstants::PONG;  // If the action code is PONG, then the ping was
                                         // successful.
    }
    return false;  // If no packet was received, then the ping failed.
}

// --- PUBLIC FUNCTIONS --- //

chainNeighborManager::chainNeighborManager(uint16_t moduleType, uint8_t* networkAddress,
                                           uint8_t hostOctet
                                               statusManager::statusManager statusManager,
                                           EthernetUDP sysAdmin, uint8_t* generalBuffer) {
    this->moduleType = moduleType;
    this->NetworkAddress[0] = networkAddress[0];
    this->NetworkAddress[1] = networkAddress[1];
    this->NetworkAddress[2] = networkAddress[2];
    this->NetworkAddress[3] = networkAddress[3];
    this->hostOctet = hostOctet;
    this->NeighborOctet = 0;
    this->statusManager = statusManager;
    this->sysAdmin = sysAdmin;
    this->generalBuffer = generalBuffer;

    chainNeighborConnected = false;
    chainOperational = false;
    timeUntilChainCheck = 0;
    lastOctetChecked =
        hostAddressOctet + 1;  // Start checking the chain at the next module in the chain
}

chainNeighborManager::~chainNeighborManager() {
    statusManager.notifyChainNeighborStatus(
        false, false);  // Call the callback function to notify the statusManager that the chain
                        // neighbor is no longer connected As the chainNeighborManager is being
                        // destroyed, the chain neighbor is no longer connected
}

bool chainNeighborManager::getChainNeighborConnected() { return chainNeighborConnected; }

bool chainNeighborManager::getChainOperational() { return chainOperational; }

uint8_t chainNeighborManager::getChainNeighborOctet() { return NeighborOctet; }

void chainNeighborManager::discoverChain() {
    // DiscoverChain is an update function that gives the module a chance to discover its chain
    // neighbors, and manage the chain. This function should be called periodically, but must not
    // interrupt activity on the sysadmin UDP port.  This will cause issues. IE don't run it in an
    // ISR.

    if (chainNeighborConnected && chainOperational && timeUntilChainCheck > 0) {
        // If the chain neighbor is connected and operational, then we are good to go.
        // Lets ping the chain neighbor to make sure it is still there. No need to check the whole
        // chain, just the neighbor.

        if (!pingModule(NeighborOctet)) {
            // If the ping fails, then the chain neighbor is no longer connected.
            chainNeighborConnected = false;
            chainOperational = false;
            statusManager.notifyChainNeighborStatus(
                chainNeighborConnected,
                chainOperational);  // Call the callback function to notify the statusManager that
                                    // the chain neighbor is no longer connected
        }
        // If the ping is successful, then the chain neighbor is still connected. No need to do
        // anything.

        timeUntilChainCheck--;  // Decrement the time until the chain is checked again.
        lastOctetChecked =
            hostOctet + 1;  // Start checking the chain at the next module in the chain
        return;             // Finished with this cycle, give main process the CPU back.

    } else if (chainNeighborConnected && chainOperational && timeUntilChainCheck == 0) {
        // Everything seems to be working fine, but it is time to check the chain again.
        // This is important to make sure no modules have been added in between this module and it's
        // neighbor. The entire chain will be checked too.

    } else if (chainNeighborConnected && (!chainOperational || timeUntilChainCheck == 0)) {
        // The chain neighbor is connected, but the chain is not operational, or it is time to check
        // the chain. Just check the whole chain. Don't worry about searching for a closer neighbor.
        // That will be done in the next cycle.

    } else {
        // The chain neighbor is not connected, so we need to discover the chain.
        // We will start by pinging the next module in the chain.

        // Increment the lastOctetChecked to check the next module in the chain, and skip this
        // modules octet.
        lastOctetChecked++;
        if (lastOctetChecked == hostOctet) {
            lastOctetChecked++;
        }

        if (!pingModule(lastOctetChecked)) {
            // If the ping fails, then the chain neighbor is not connected.
            // No need to update state, as nothing has changed.
            return;  // give up on this cycle, give main process the CPU back.
        } else {
            // If the ping is successful, then a chain neighbor is connected.
            chainNeighborConnected = true;
            chainOperational = true;
            NeighborOctet = lastOctetChecked;
            statusManager.notifyChainNeighborStatus(
                chainNeighborConnected,
                chainOperational);  // Call the callback function to notify the statusManager that
                                    // the chain neighbor is connected
        }
    }
}

bool chainNeighborManager::chainForward(ROIPackets::sysAdminPacket packet) {
    // ChainForward is a function that forwards a packet to the next module in the chain.
    // sysAdminHandler will call this function, and should send a "forward packet" to the next
    // module in the chain.

    IPAddress forwardIP =
        IPAddress(NetworkAddress[0], NetworkAddress[1], NetworkAddress[2], NeighborOctet);

    packet.exportPacket(generalBuffer,
                        ROIConstants::ROIMAXPACKETSIZE);  // Export the packet to the general buffer

    sysAdmin.beginPacket(forwardIP, ROIConstants::ROISYSADMINPORT);
    sysAdmin.write(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);
    return sysAdmin.endPacket();
}