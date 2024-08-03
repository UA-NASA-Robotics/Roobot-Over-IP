#include "chainNeighborManager.h"

// --- PRIVATE FUNCTIONS --- //

bool chainNeighborManager::chainNeighborManager::pingModule(uint8_t clientAddressOctet) {
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
    if (!sysAdmin.endPacket()) {
        // If the packet fails to send, then the chain neighbor is no longer connected.
        return false;
    }

    // now wait for a response from the chain neighbor
    long startTime = millis();
    int packetReceived = 0;
    while (millis() - startTime < chainManagerConstants::CHAINTIMEOUT && !packetReceived) {
        if (sysAdmin.parsePacket()) {  // A packet was received from module, check it is coherent.
            sysAdmin.read(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);

            ROIPackets::sysAdminPacket responsePacket;
            if (!responsePacket.importPacket(generalBuffer, ROIConstants::ROIMAXPACKETSIZE)) {
                return false;  // If the packet is not coherent, then the ping failed.
            };
            if (responsePacket.getActionCode() ==
                sysAdminConstants::PONG)  // If the action code is PONG, then the ping was
                                          // successful.
            {
                return true;
            }

            // else the packet was not a PONG, so we ignore it and wait for the next packet.
            //(Yes this may eat packets, but it is the responsibility of the sender to resend if
            // needed)
        }
    }

    return false;  // If no packet was received, then the ping failed.
}

int16_t chainNeighborManager::chainNeighborManager::pingChain() {
    // Ping the entire chain to make sure it is still there, True if the ping is successful
    ROIPackets::sysAdminPacket pingPacket;  // Create a sysAdminPacket object that will be used
                                            // to ping the module
    IPAddress moduleIP(NetworkAddress[0], NetworkAddress[1], NetworkAddress[2],
                       neighborOctet);  // Create an IPAddress object for the module
    pingPacket.setHostAddressOctet(hostOctet);
    pingPacket.setClientAddressOctet(neighborOctet);
    pingPacket.setAdminMetaData(
        sysAdminConstants::CHAINMESSAGEMETA ||
        hostOctet);  // Set the metadata to chain message that reply's back to this module
    pingPacket.setActionCode(sysAdminConstants::PING);

    pingPacket.exportPacket(
        generalBuffer,
        ROIConstants::ROIMAXPACKETSIZE);  // Export the packet to the general buffer

    sysAdmin.beginPacket(moduleIP,
                         ROIConstants::ROISYSADMINPORT);  // Send the ping packet to the module
    sysAdmin.write(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);
    if (!sysAdmin.endPacket()) {
        // If the packet fails to send, then the chain neighbor is no longer connected.
        return -1;
    }

    // now get responses from the chain

    long startTime = millis();
    uint8_t chainLength =
        0;  // The chain length is the number of modules in the chain (Discovery only work on a
            // single subnet, so the chain length is limited to 255)
    while (millis() - startTime < chainManagerConstants::CHAINTIMEOUT) {
        if (sysAdmin.parsePacket()) {  // A packet was received from module, check it is coherent.
            sysAdmin.read(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);

            ROIPackets::sysAdminPacket responsePacket;
            if (!responsePacket.importPacket(generalBuffer, ROIConstants::ROIMAXPACKETSIZE)) {
                return -1;  // If the packet is not coherent, then the ping failed.
            };
            if (responsePacket.getActionCode() ==
                sysAdminConstants::PONG)  // If the action code is PONG, then the ping was
                                          // successful.
            {
                chainLength++;  // Increment the chain length for each module in the chain
            }
            if (responsePacket.getActionCode() == sysAdminConstants::PINGLOOPBACK) {
                return chainLength;  // If the action code is PINGLOOPBACK, then the chain is
                                     // complete
            }

            // else the packet was not a PONG, so we ignore it and wait for the next packet.
            //(Yes this may eat packets, but it is the responsibility of the sender to resend if
            // needed)
        }
    }

    return -1;  // If no packet was received, then the ping failed.
}

uint8_t chainNeighborManager::chainNeighborManager::pingRangeMinima(uint8_t minimumOctet,
                                                                    uint8_t maximumOctet) {
    // Ping a range of octets, returns the minima octet
    ROIPackets::sysAdminPacket pingPacket;  // Create a sysAdminPacket object that will be used
    // to ping
    pingPacket.setHostAddressOctet(hostOctet);
    pingPacket.setAdminMetaData(sysAdminConstants::NOCHAINMETA);
    pingPacket.setActionCode(sysAdminConstants::PING);
    pingPacket.exportPacket(
        generalBuffer,
        ROIConstants::ROIMAXPACKETSIZE);  // Export the packet to the general buffer

    bool wrapped = maximumOctet < minimumOctet;  // Check if the range wraps around 0
    for (uint8_t i = minimumOctet; i < maximumOctet && wrapped;
         i++) {  // Loop through the range of octets and send a ping packet to each module
        IPAddress moduleIP(NetworkAddress[0], NetworkAddress[1], NetworkAddress[2],
                           i);  // Create an IPAddress object for the module

        sysAdmin.beginPacket(moduleIP,
                             ROIConstants::ROISYSADMINPORT);  // Send the ping packet to the module
        sysAdmin.write(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);
        sysAdmin.endPacket();  // Send the packet (it may time out, but that is okay, no need to
                               // check)

        delay(1);  // Delay to prevent flooding the network

        if (i == 255) {
            wrapped =
                false;  // we have looped around the range once, so we can stop at maximumOctet now
                        // i will (hopefully) overflow to 0, and then increment to maximumOctets
        }
    }

    // now wait for a response from the chain neighbor
    long startTime = millis();
    uint8_t minimaOctet = 255;  // The minima octet is the smallest octet in the range
    while (millis() - startTime < chainManagerConstants::CHAINTIMEOUT) {
        if (sysAdmin.parsePacket()) {  // A packet was received from module, check it is coherent.
            IPAddress moduleIP = sysAdmin.remoteIP();
            sysAdmin.read(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);

            ROIPackets::sysAdminPacket responsePacket;

            if (!responsePacket.importPacket(generalBuffer, ROIConstants::ROIMAXPACKETSIZE)) {
                continue;  // If the packet is not coherent, then ignore it and wait for the next
                           // packet.
            }

            if (responsePacket.getActionCode() ==
                    sysAdminConstants::PONG)  // If the action code is PONG, then the ping was
                                               // successful.
            {
                if (moduleIP[3] < minimaOctet) {
                    minimaOctet = moduleIP[3];  // If the octet is smaller than the minima octet,
                                                // then update the minima octet
                }
                if (moduleIP[3] == minimumOctet) {
                    sysAdmin.flush();  // If the octet is the minimum octet, then flush the buffer
                                       // of any other packets (this may or may not work. check the
                                       // source code bcs this library is bad)
                    return minimaOctet;  // If the octet is the minimum octet, then the range is
                                         // complete
                }
            }

            // else the packet was not a PONG, so we ignore it and wait for the next packet.
            //(Yes this may eat packets, but it is the responsibility of the sender to resend if
            // needed)
        }
    }

    return 255;  // If no packet was received, then the ping failed.
}

// --- PUBLIC FUNCTIONS --- //

chainNeighborManager::chainNeighborManager::chainNeighborManager() {
    // Default constructor
    chainNeighborConnected = false;
    chainOperational = false;
    timeUntilChainCheck = 0;
    lastOctetChecked = 0;
}

chainNeighborManager::chainNeighborManager::chainNeighborManager(
    uint16_t moduleType, uint8_t* networkAddress, uint8_t hostOctet,
    statusManager::statusManager moduleStatusManager, EthernetUDP sysAdmin,
    uint8_t* generalBuffer) {
    this->moduleType = moduleType;
    this->NetworkAddress[0] = networkAddress[0];
    this->NetworkAddress[1] = networkAddress[1];
    this->NetworkAddress[2] = networkAddress[2];
    this->NetworkAddress[3] = networkAddress[3];
    this->hostOctet = hostOctet;
    this->neighborOctet = 0;
    this->statusManager = moduleStatusManager;
    this->sysAdmin = sysAdmin;
    this->generalBuffer = generalBuffer;

    chainNeighborConnected = false;
    chainOperational = false;
    timeUntilChainCheck = 0;
    lastOctetChecked = hostOctet + 1;  // Start checking the chain at the next module in the chain
}

chainNeighborManager::chainNeighborManager::~chainNeighborManager() {
    statusManager.notifyChainNeighborStatus(
        false, false);  // Call the callback function to notify the statusManager that the chain
                        // neighbor is no longer connected As the chainNeighborManager is being
                        // destroyed, the chain neighbor is no longer connected
}

bool chainNeighborManager::chainNeighborManager::getChainNeighborConnected() {
    return chainNeighborConnected;
}

bool chainNeighborManager::chainNeighborManager::getChainOperational() { return chainOperational; }

uint8_t chainNeighborManager::chainNeighborManager::getChainNeighborOctet() {
    return neighborOctet;
}

void chainNeighborManager::chainNeighborManager::discoverChain() {
    // DiscoverChain is an update function that gives the module a chance to discover its chain
    // neighbors, and manage the chain. This function should be called periodically, but must
    // not interrupt activity on the sysadmin UDP port.  This will cause issues. IE don't run it
    // in an ISR.

    if (chainNeighborConnected && chainOperational && timeUntilChainCheck > 0) {
        // If the chain neighbor is connected and operational, then we are good to go.
        // Lets ping the chain neighbor to make sure it is still there. No need to check the
        // whole chain, just the neighbor.

        if (!pingModule(neighborOctet)) {
            // If the ping fails, then the chain neighbor is no longer connected.
            chainNeighborConnected = false;
            chainOperational = false;
            statusManager.notifyChainNeighborStatus(
                chainNeighborConnected,
                chainOperational);  // Call the callback function to notify the statusManager
                                    // that the chain neighbor is no longer connected
        }
        // If the ping is successful, then the chain neighbor is still connected. No need to do
        // anything.

        timeUntilChainCheck--;  // Decrement the time until the chain is checked again.
        lastOctetChecked =
            hostOctet + 1;  // Start checking the chain at the next module in the chain
        return;             // Finished with this cycle, give main process the CPU back.

    } else if (chainNeighborConnected && chainOperational && timeUntilChainCheck == 0) {
        // Everything seems to be working fine, but it is time to check the chain again.
        // This is important to make sure no modules have been added in between this module and
        // it's neighbor. The entire chain will be checked too.

        int16_t chainLength = pingChain();  // Ping the entire chain to make sure it is still there

        if (chainLength == -1) {
            // If the chain is broken, then the chain neighbor is no longer connected.
            chainOperational = false;
            statusManager.notifyChainNeighborStatus(
                chainNeighborConnected,
                chainOperational);  // Call the callback function to notify the statusManager
                                    // that the chain neighbor is no longer connected
            return;                 // give up on this cycle, give main process the CPU back.
        }

        uint8_t minimaOctet = pingRangeMinima(
            hostOctet + 1,
            neighborOctet);  // Ping the range of octets to find the next module in the chain

        if (minimaOctet == 255) {
            // if the minima is 255, no intermediary modules were discovered, so the chain is intact
            // and unchanged
            timeUntilChainCheck =
                chainManagerConstants::CHAINCHECKINTERVAL;  // Reset the time until the chain is
                                                            // checked again
            return;  // end this cycle, give main process the CPU back.

        } else {
            // If the minima is not 255, then an intermediary module was discovered, and the chain
            // has changed. The chain neighbor is no longer connected. Force a re-discovery next
            // isr.
            chainOperational = false;
            chainNeighborConnected = false;
            statusManager.notifyChainNeighborStatus(
                chainNeighborConnected,
                chainOperational);  // Call the callback function to notify the statusManager
                                    // that the chain neighbor is no longer connected
            return;                 // give up on this cycle, give main process the CPU back.
        }
    } else if (chainNeighborConnected && (!chainOperational || timeUntilChainCheck == 0)) {
        // The chain neighbor is connected, but the chain is not operational, or it is time to
        // check the chain. Just check the whole chain. Don't worry about searching for a closer
        // neighbor. That will be done in the next cycle.

        int16_t chainLength = pingChain();  // Ping the entire chain to make sure it is still there

        if (chainLength > -1) {
            // If the chain is operational, then the chain neighbor is still connected.
            chainOperational = true;
            timeUntilChainCheck = chainManagerConstants::CHAINCHECKINTERVAL;  // Reset the time
                                                                              // until the chain
                                                                              // is checked again
            statusManager.notifyChainNeighborStatus(
                chainNeighborConnected,
                chainOperational);  // Call the callback function to notify the statusManager
                                    // that the chain neighbor is connected
            return;                 // end this cycle, give main process the CPU back.
        }  // else no need to update state, as nothing has changed.

    } else {
        // The chain neighbor is not connected or this FSM is in a broken sate, so we need to
        // discover the chain. We will start by pinging the next module in the chain.

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
            chainOperational =
                false;  // The chain is not operational until the entire chain is checked
            neighborOctet = lastOctetChecked;
            statusManager.notifyChainNeighborStatus(
                chainNeighborConnected,
                chainOperational);  // Call the callback function to notify the statusManager
                                    // that the chain neighbor is connected
        }
    }
}

bool chainNeighborManager::chainNeighborManager::chainForward(ROIPackets::sysAdminPacket packet) {
    // ChainForward is a function that forwards a packet to the next module in the chain.
    // sysAdminHandler will call this function, and should send a "forward packet" to the next
    // module in the chain.

    IPAddress forwardIP =
        IPAddress(NetworkAddress[0], NetworkAddress[1], NetworkAddress[2], neighborOctet);

    packet.exportPacket(generalBuffer,
                        ROIConstants::ROIMAXPACKETSIZE);  // Export the packet to the general buffer

    sysAdmin.beginPacket(forwardIP, ROIConstants::ROISYSADMINPORT);
    sysAdmin.write(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);
    return sysAdmin.endPacket();
}