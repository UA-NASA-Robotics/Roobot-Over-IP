#include "chainNeighborManager.h"

// --- PRIVATE FUNCTIONS --- //

bool chainNeighborManager::chainNeighborManager::_pingModule(uint8_t clientAddressOctet) {
    // Ping the chain neighbor to make sure it is still there, True if the ping is successful

#if DEBUG && defined(__AVR__)
    Serial.print(F("Ping module: "));
    Serial.println(clientAddressOctet);
#endif
    ROIPackets::sysAdminPacket pingPacket;  // Create a sysAdminPacket object that will be used
                                            // to ping the module
    IPAddress moduleIP(_ipContainer.addressArray[0], _ipContainer.addressArray[1],
                       _ipContainer.addressArray[2],
                       clientAddressOctet);  // Create an IPAddress object for the module
    pingPacket.setHostAddressOctet(_ipContainer.addressArray[3]);
    pingPacket.setClientAddressOctet(clientAddressOctet);
    pingPacket.setAdminMetaData(sysAdminConstants::NO_CHAIN_META);
    pingPacket.setActionCode(sysAdminConstants::PING);

    pingPacket.exportPacket(
        _generalBuffer,
        ROIConstants::ROI_MAX_PACKET_SIZE);  // Export the packet to the general buffer

    if (!_sysAdmin.beginPacket(
            moduleIP,
            ROIConstants::ROI_SYS_ADMIN_PORT)) {  // Send the ping packet to the module
#if DEBUG && defined(__AVR__)
        Serial.println(F("Ping module, packet failed to begin, invalid IP"));
#endif
        return false;
    }
    _sysAdmin.write(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);
    if (!_sysAdmin.endPacket()) {
        // If the packet fails to send, then the chain neighbor is no longer connected.
#if DEBUG && defined(__AVR__)
        Serial.println(F("Ping module, packet failed to send"));
#endif
        return false;
    }

    // now wait for a response from the chain neighbor
    unsigned long startTime = millis();
    while (millis() - startTime < chainManagerConstants::CHAIN_TIME_OUT) {
        if (_sysAdmin.parsePacket()) {  // A packet was received from module, check it is coherent.
            _sysAdmin.read(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);

            ROIPackets::sysAdminPacket responsePacket;
            if (!responsePacket.importPacket(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE)) {
#if DEBUG && defined(__AVR__)
                Serial.println(F("Ping module, packet failed to import"));
#endif
                return false;  // If the packet is not coherent, then the ping failed.
            };
            if (responsePacket.getActionCode() ==
                sysAdminConstants::PONG)  // If the action code is PONG, then the ping was
                                          // successful.
            {
#if DEBUG && defined(__AVR__)
                Serial.println(F("Ping module, pong received"));
#endif
                return true;
            }

            // else the packet was not a PONG, so we ignore it and wait for the next packet.
            //(Yes this may eat packets, but it is the responsibility of the sender to resend if
            // needed)
        }
    }

#if DEBUG && defined(__AVR__)
    Serial.println(F("Ping module, timeout"));
#endif
    return false;  // If no packet was received, then the ping failed.
}

int16_t chainNeighborManager::chainNeighborManager::_pingChain() {
    // Ping the entire chain to make sure it is still there, True if the ping is successful

#if DEBUG && defined(__AVR__)
    Serial.println(F("Ping chain"));
#endif

    ROIPackets::sysAdminPacket pingPacket;  // Create a sysAdminPacket object that will be used
                                            // to ping the module
    IPAddress moduleIP(_ipContainer.addressArray[0], _ipContainer.addressArray[1],
                       _ipContainer.addressArray[2],
                       _neighborOctet);  // Create an IPAddress object for the module
    pingPacket.setHostAddressOctet(_ipContainer.addressArray[3]);
    pingPacket.setClientAddressOctet(_neighborOctet);
    pingPacket.setAdminMetaData(sysAdminConstants::CHAIN_MESSAGE_META ||
                                _ipContainer.addressArray[3]);  // Set the metadata to chain message
                                                                // that reply's back to this module
    pingPacket.setActionCode(sysAdminConstants::PING);

    pingPacket.exportPacket(
        _generalBuffer,
        ROIConstants::ROI_MAX_PACKET_SIZE);  // Export the packet to the general buffer

    _sysAdmin.beginPacket(moduleIP,
                          ROIConstants::ROI_SYS_ADMIN_PORT);  // Send the ping packet to the module
    _sysAdmin.write(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);
    if (!_sysAdmin.endPacket()) {
// If the packet fails to send, then the chain neighbor is no longer connected.
#if DEBUG && defined(__AVR__)
        Serial.println(F("Ping chain, packet failed to send"));
#endif
        return -1;
    }

    // now get responses from the chain

    long startTime = millis();
    uint8_t chainLength =
        0;  // The chain length is the number of modules in the chain (Discovery only work on a
            // single subnet, so the chain length is limited to 255)
    while (millis() - startTime < chainManagerConstants::CHAIN_TIME_OUT) {
        if (_sysAdmin.parsePacket()) {  // A packet was received from module, check it is coherent.
            _sysAdmin.read(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);

            ROIPackets::sysAdminPacket responsePacket;
            if (!responsePacket.importPacket(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE)) {
#if DEBUG && defined(__AVR__)
                Serial.println(F("Ping chain, packet failed to import"));
#endif
                // return -1;  // If the packet is not coherent, then the ping failed.
                continue;  //(be forgiving, and just ignore the packet)
            };
            if (responsePacket.getActionCode() ==
                sysAdminConstants::PONG)  // If the action code is PONG, then the ping was
                                          // successful.
            {
#if DEBUG && defined(__AVR__)
                Serial.println(F("Ping chain, pong received"));
#endif
                chainLength++;  // Increment the chain length for each module in the chain
                continue;
            }
            if (responsePacket.getActionCode() == sysAdminConstants::PING_LOOP_BACK) {
#if DEBUG && defined(__AVR__)
                Serial.println(F("Ping chain, loopback received"));
#endif
                return chainLength;  // If the action code is PING_LOOP_BACK, then the chain is
                                     // complete
            }

            // else the packet was not a PONG, so we ignore it and wait for the next packet.
            //(Yes this may eat packets, but it is the responsibility of the sender to resend if
            // needed)
        }
    }
#if DEBUG && defined(__AVR__)
    Serial.println(F("Ping chain, timeout"));
#endif
    return -1;  // If no packet was received, then the ping failed.
}

uint16_t chainNeighborManager::chainNeighborManager::_pingRangeMinima(uint8_t minimumOctet,
                                                                      uint8_t maximumOctet) {
    // Ping a range of octets, returns the minima octet

#if DEBUG && defined(__AVR__)
    Serial.print(F("Ping range minima: "));
    Serial.print(minimumOctet);
    Serial.print(F(" to "));
    Serial.println(maximumOctet);
#endif

    if (minimumOctet == maximumOctet) {
        return chainManagerConstants::NULL_OCTET;  // If the range is only one octet, then the
                                                   // minima is the octet
    }

    ROIPackets::sysAdminPacket pingPacket;  // Create a sysAdminPacket object that will be used
    // to ping
    pingPacket.setHostAddressOctet(_ipContainer.addressArray[3]);
    pingPacket.setAdminMetaData(sysAdminConstants::NO_CHAIN_META);
    pingPacket.setActionCode(sysAdminConstants::PING);
    pingPacket.exportPacket(
        _generalBuffer,
        ROIConstants::ROI_MAX_PACKET_SIZE);  // Export the packet to the general buffer

    bool wrapped = maximumOctet < minimumOctet;  // Check if the range wraps around 0
    for (uint8_t i = minimumOctet; i < maximumOctet && wrapped;
         i++) {  // Loop through the range of octets and send a ping packet to each module

#if DEBUG && defined(__AVR__)
        Serial.print(F("Ping range minima, ping module: "));
        Serial.println(i);
#endif

        IPAddress moduleIP(_ipContainer.addressArray[0], _ipContainer.addressArray[1],
                           _ipContainer.addressArray[2],
                           i);  // Create an IPAddress object for the module

        _sysAdmin.beginPacket(
            moduleIP,
            ROIConstants::ROI_SYS_ADMIN_PORT);  // Send the ping packet to the module
        _sysAdmin.write(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);
        _sysAdmin.endPacket();  // Send the packet (it may time out, but that is okay, no need to
                                // check)

        delay(5);  // Delay to prevent flooding the network

        if (i == 255) {
            wrapped = false;  // we have looped around the range once, so we can stop at
                              // maximumOctet now i will (hopefully) overflow to 0, and then
                              // increment to maximumOctets
        }
    }

    // now wait for a response from the chain neighbor
    long startTime = millis();
    uint16_t minimaOctet = chainManagerConstants::NULL_OCTET;  // The minima octet is the
                                                               // smallest octet in the range
    while (millis() - startTime < chainManagerConstants::CHAIN_TIME_OUT) {
        if (_sysAdmin.parsePacket()) {  // A packet was received from module, check it is coherent.
            IPAddress moduleIP = _sysAdmin.remoteIP();
            _sysAdmin.read(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);

            ROIPackets::sysAdminPacket responsePacket;

            if (!responsePacket.importPacket(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE)) {
                continue;  // If the packet is not coherent, then ignore it and wait for the
                           // next
// packet.
#if DEBUG && defined(__AVR__)
                Serial.print(F("Ping range minima, packet failed to import from: "));
                Serial.println(moduleIP[3]);
#endif
            }

            if (responsePacket.getActionCode() ==
                sysAdminConstants::PONG)  // If the action code is PONG, then the ping was
                                          // successful.
            {
#if DEBUG && defined(__AVR__)
                Serial.print(F("Ping range minima, pong received from: "));
                Serial.println(moduleIP[3]);
#endif
                if (moduleIP[3] < minimaOctet) {
                    minimaOctet = moduleIP[3];  // If the octet is smaller than the minima
                                                // octet, then update the minima octet
                }

                if (moduleIP[3] == minimumOctet) {
                    _sysAdmin.flush();  // If the octet is the minimum octet, then flush the buffer
// of any other packets (this may or may not work. check the
// source code bcs this library is bad)
#if DEBUG && defined(__AVR__)
                    Serial.print(F("Ping range minima, minima found:"));
                    Serial.println(minimaOctet);
#endif
                    return minimaOctet;  // If the octet is the minimum octet, then the range is
                                         // complete
                }
            }

            // else the packet was not a PONG, so we ignore it and wait for the next packet.
            //(Yes this may eat packets, but it is the responsibility of the sender to resend if
            // needed)
        }
    }

#if DEBUG && defined(__AVR__)
    Serial.println(F("Ping range minima, timeout"));
#endif
    return minimaOctet;  // If timeout, then return the minima octet,
                         // chainManagerConstants::NULL_OCTET = none found but any valid octet is
                         // the minima
}

// --- PUBLIC FUNCTIONS --- //

chainNeighborManager::chainNeighborManager::chainNeighborManager(
    uint16_t moduleType, IPContainer& IPContainer,
    statusManager::statusManager& moduleStatusManager, EthernetUDP& sysAdmin,
    uint8_t* generalBuffer)
    : _statusManager(moduleStatusManager),
      _sysAdmin(sysAdmin),
      _generalBuffer(generalBuffer),
      _ipContainer(IPContainer) {
    this->_moduleType = moduleType;
    this->_neighborOctet = 0;

    _chainNeighborConnected = false;
    _chainOperational = false;
    _timeUntilChainCheck = 0;
    _lastOctetChecked = 2;  // Start checking the chain at the next module in the chain
}

chainNeighborManager::chainNeighborManager::~chainNeighborManager() {
    _statusManager.notifyChainNeighborStatus(
        false, false);  // Call the callback function to notify the statusManager that the chain
                        // neighbor is no longer connected As the chainNeighborManager is being
                        // destroyed, the chain neighbor is no longer connected
}

bool chainNeighborManager::chainNeighborManager::getChainNeighborConnected() {
    return _chainNeighborConnected;
}

bool chainNeighborManager::chainNeighborManager::getChainOperational() { return _chainOperational; }

uint8_t chainNeighborManager::chainNeighborManager::getChainNeighborOctet() {
    return _neighborOctet;
}

void chainNeighborManager::chainNeighborManager::notifyDoDiscovery() {
    _doDiscovery = true;  // Set the doDiscovery flag to true
}

void chainNeighborManager::chainNeighborManager::discoverChain() {
    // DiscoverChain is an update function that gives the module a chance to discover its chain
    // neighbors, and manage the chain. This function should be called periodically, but must
    // not interrupt activity on the sysadmin UDP port.  This will cause issues. IE don't run it
    // in an ISR.

    if (!_doDiscovery) {
        return;  // If the doDiscovery flag is not set, then there is no need to discover the
                 // chain on this cycle
    }

    _doDiscovery = false;  // Reset the doDiscovery flag

    if (_chainNeighborConnected && _chainOperational && _timeUntilChainCheck > 0) {
        // If the chain neighbor is connected and operational, then we are good to go.
        // Lets ping the chain neighbor to make sure it is still there. No need to check the
        // whole chain, just the neighbor.

#if DEBUG && defined(__AVR__)
        Serial.println(F("Discover chain, ping chain neighbor"));
#endif

        if (!_pingModule(_neighborOctet)) {
            // If the ping fails, then the chain neighbor is no longer connected.
            _chainNeighborConnected = false;
            _chainOperational = false;
            _statusManager.notifyChainNeighborStatus(
                _chainNeighborConnected,
                _chainOperational);  // Call the callback function to notify the statusManager
                                     // that the chain neighbor is no longer connected

#if DEBUG && defined(__AVR__)
            Serial.println(F("Discover chain, ping chain neighbor failed"));
#endif
        }
        // If the ping is successful, then the chain neighbor is still connected. No need to do
        // anything.

        _timeUntilChainCheck--;  // Decrement the time until the chain is checked again.
        _lastOctetChecked =
            _ipContainer
                .networkAddress[3];  // Start checking the chain at the next module in the chain
        return;                      // Finished with this cycle, give main process the CPU back.

    } else if (_chainNeighborConnected && _chainOperational && _timeUntilChainCheck == 0) {
        // Everything seems to be working fine, but it is time to check the chain again.
        // This is important to make sure no modules have been added in between this module and
        // it's neighbor. The entire chain will be checked too.

#if DEBUG && defined(__AVR__)
        Serial.println(F("Discover chain, check chain and discover intermediary modules"));
#endif

        int16_t chainLength = _pingChain();  // Ping the entire chain to make sure it is still there

        if (chainLength == -1) {
            // If the chain is broken, then the chain neighbor is no longer connected.
            _chainOperational = false;
            _statusManager.notifyChainNeighborStatus(
                _chainNeighborConnected,
                _chainOperational);  // Call the callback function to notify the statusManager
                                     // that the chain neighbor is no longer connected
            return;                  // give up on this cycle, give main process the CPU back.

#if DEBUG && defined(__AVR__)
            Serial.println(F("Discover chain, check chain failed"));
#endif
        }

        uint8_t minimaOctet = _pingRangeMinima(
            _ipContainer.addressArray[3] + 1,
            _neighborOctet);  // Ping the range of octets to find the next module in the chain

        if (minimaOctet == chainManagerConstants::NULL_OCTET) {
            // if the minima is chainManagerConstants::NULL_OCTET, no intermediary modules were
            // discovered, so the chain is intact and unchanged
            _timeUntilChainCheck =
                chainManagerConstants::CHAIN_CHECK_INTERVAL;  // Reset the time until the chain is
                                                              // checked again

#if DEBUG && defined(__AVR__)
            Serial.println(F("Discover chain, no intermediary modules found"));
#endif
            return;  // end this cycle, give main process the CPU back.

        } else {
            // If the minima is not 255, then an intermediary module was discovered, and the
            // chain has changed. The chain neighbor is no longer connected. Force a
            // re-discovery next isr.
            _chainOperational = false;
            _chainNeighborConnected = false;
            _statusManager.notifyChainNeighborStatus(
                _chainNeighborConnected,
                _chainOperational);  // Call the callback function to notify the statusManager
                                     // that the chain neighbor is no longer connected

#if DEBUG && defined(__AVR__)
            Serial.print(F("Discover chain, intermediary module found: "));
            Serial.println(minimaOctet);
#endif
            return;  // give up on this cycle, give main process the CPU back.
        }
    } else if (_chainNeighborConnected && (!_chainOperational || _timeUntilChainCheck == 0)) {
        // The chain neighbor is connected, but the chain is not operational, or it is time to
        // check the chain. Just check the whole chain. Don't worry about searching for a closer
        // neighbor. That will be done in the next cycle.

#if DEBUG && defined(__AVR__)
        Serial.println(F("Discover chain, check chain"));
#endif

        int16_t chainLength = _pingChain();  // Ping the entire chain to make sure it is still there

        if (chainLength > -1) {
            // If the chain is operational, then the chain neighbor is still connected.
            _chainOperational = true;
            _timeUntilChainCheck = chainManagerConstants::CHAIN_CHECK_INTERVAL;  // Reset the time
                                                                                 // until the chain
                                                                                 // is checked again
            _statusManager.notifyChainNeighborStatus(
                _chainNeighborConnected,
                _chainOperational);  // Call the callback function to notify the statusManager
                                     // that the chain neighbor is connected

#if DEBUG && defined(__AVR__)
            Serial.println(F("Discover chain, check chain successful"));
#endif

            return;  // end this cycle, give main process the CPU back.
        }  // else no need to update state, as nothing has changed.

    } else {
        // The chain neighbor is not connected or this FSM is in a broken sate, so we need to
        // discover the chain. We will start by pinging the next module in the chain.

#if DEBUG && defined(__AVR__)
        Serial.println(F("Discover chain, discover chain neighbor"));
#endif

        // Increment the lastOctetChecked to check the next module in the chain, and skip this
        // modules octet.
        _lastOctetChecked++;
        if (_lastOctetChecked == _ipContainer.addressArray[3]) {
            _lastOctetChecked++;
        } else if (_lastOctetChecked == 255)  // If we have reached the end of the octet range, wrap
                                              // around to 1 (255 is the broadcast octet)
        {
            _lastOctetChecked = 1;  // Wrap around to 1, as 0 is the null octet
        }

        if (!_pingModule(_lastOctetChecked)) {
            // If the ping fails, then the chain neighbor is not connected.
            // No need to update state, as nothing has changed.
#if DEBUG && defined(__AVR__)
            Serial.println(F("Discover chain, discover chain neighbor failed"));
#endif
            return;  // give up on this cycle, give main process the CPU back.
        } else {
            // If the ping is successful, then a chain neighbor is connected.
            _chainNeighborConnected = true;
            _chainOperational =
                false;  // The chain is not operational until the entire chain is checked
            _neighborOctet = _lastOctetChecked;
            _statusManager.notifyChainNeighborStatus(
                _chainNeighborConnected,
                _chainOperational);  // Call the callback function to notify the statusManager
                                     // that the chain neighbor is connected

#if DEBUG && defined(__AVR__)
            Serial.print(F("Discover chain, discover chain neighbor successful: "));
            Serial.println(_neighborOctet);
#endif

            return;  // end this cycle, give main process the CPU back.
        }
    }
}

bool chainNeighborManager::chainNeighborManager::chainForward(ROIPackets::sysAdminPacket packet) {
    // ChainForward is a function that forwards a packet to the next module in the chain.
    // sysAdminHandler will call this function, and should send a "forward packet" to the next
    // module in the chain.

#if DEBUG && defined(__AVR__)
    Serial.println(F("Chain forward"));
#endif

    if (!_chainNeighborConnected || !_chainOperational) {
        return false;  // If the chain neighbor is not connected or the chain is not
                       // operational, then the packet cannot be forwarded
    }

    IPAddress forwardIP = IPAddress(_ipContainer.networkAddress[0], _ipContainer.networkAddress[1],
                                    _ipContainer.networkAddress[2], _neighborOctet);

    packet.exportPacket(
        _generalBuffer,
        ROIConstants::ROI_MAX_PACKET_SIZE);  // Export the packet to the general buffer

    _sysAdmin.beginPacket(forwardIP, ROIConstants::ROI_SYS_ADMIN_PORT);
    _sysAdmin.write(_generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);
    return _sysAdmin.endPacket();
}