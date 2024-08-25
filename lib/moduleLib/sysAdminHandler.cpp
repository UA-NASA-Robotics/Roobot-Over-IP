#include "sysAdminHandler.h"

sysAdminHandler::sysAdminHandler::sysAdminHandler(
    uint16_t moduleType, statusManager::statusManager& statusManager,
    chainNeighborManager::chainNeighborManager& chainManager, BlacklistManager& blacklistManager,
    uint8_t* generalBuffer)
    : statusManager(statusManager),
      chainManager(chainManager),
      blacklistManager(blacklistManager),
      generalBuffer(generalBuffer) {
    this->moduleType = moduleType;
}

sysAdminHandler::sysAdminHandler::~sysAdminHandler() {}

void sysAdminHandler::sysAdminHandler::setMAC(uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        this->mac[i] = mac[i];
    }
}

ROIPackets::sysAdminPacket sysAdminHandler::sysAdminHandler::handleSysAdminPacket(
    ROIPackets::sysAdminPacket packet) {
    uint16_t metaData = packet.getAdminMetaData();  // Get the metacode from the packet
    bool chainedMessage = metaData & sysAdminConstants::CHAINMESSAGEMETA;
    metaData &= ~sysAdminConstants::CHAINMESSAGEMETA;  // Remove the chain message metadata for
                                                       // the response

    uint8_t replyHostOctet = packet.getHostAddressOctet();  // Get the host octet from the packet
                                                            // (This is the default reply address)

    // Handle forwarding chain messages
    if (chainedMessage) {
#if DEBUG && defined(__AVR__)
        Serial.println(F("Chained Message"));
#endif

        replyHostOctet = packet.getAdminMetaData() &
                         0xFF;  // Get the the reply host octet from the metadata if it is a chained
                                // message, essentially overloads the reply destination

#if DEBUG && defined(__AVR__)
        Serial.print(F("Reply Host Octet: "));
        Serial.println(replyHostOctet);
#endif

        if (packet.getOriginHostOctet() != chainManager.getChainNeighborOctet() &&
            chainManager
                .getChainNeighborConnected()) {  // if this the next module in the chain is the
                                                 // origin, then the packet has traversed the
            // loop. Do NOT Forward. Also do not forward if the chain neighbor is not connected
            ROIPackets::sysAdminPacket forwardPacket;  // Create a forward packet
            forwardPacket.setHostAddressOctet(
                packet.getClientAddressOctet());  // Set the host address octet to the client
                                                  // address octet, as we are now the host
            // forwardPacket.setClientAddressOctet(
            //    chainManager.getChainNeighborOctet());  // Set the client address octet to the
            //    next
            //  module in the chain
            // Filled in by the chainManager

            forwardPacket.setAdminMetaData(metaData);  // Set the metadata of the forward packet
            forwardPacket.setActionCode(
                packet.getActionCode());  // Set the action code of the forward packet

            packet.getData(generalBuffer, ROIConstants::ROIMAXPACKETPAYLOAD);  // Get the data from
                                                                               // the packet
            forwardPacket.setData(generalBuffer,
                                  ROIConstants::ROIMAXPACKETPAYLOAD);  // Set the data of the
                                                                       // forward packet

            chainManager.chainForward(
                forwardPacket);  // Forward the packet to the next module in the chain

#if DEBUG && defined(__AVR__)
            Serial.println(F("Forwarded original packet"));
#endif

        } else if (chainManager.getChainNeighborConnected() &&
                   packet.getActionCode() == sysAdminConstants::PING &&
                   packet.getOriginHostOctet() ==
                       chainManager.getChainNeighborOctet()) {  // If the packet is a ping and the
                                                                // loop is done, send a PINGLOOPBACK
                                                                // back to the origin to acknowledge
                                                                // the loop is complete
            ROIPackets::sysAdminPacket forwardPacket;           // Create a forward packet
            forwardPacket.setHostAddressOctet(
                packet.getClientAddressOctet());  // Set the host address octet to the client
                                                  // address octet, as we are now the host
            // forwardPacket.setClientAddressOctet(
            //    chainManager.getChainNeighborOctet());  // Set the client address octet to the
            //    next
            //  module in the chain
            // Filled in by the chainManager

            forwardPacket.setAdminMetaData(metaData);  // Set the metadata of the forward packet
            forwardPacket.setActionCode(
                sysAdminConstants::PINGLOOPBACK);  // Set the action code of the forward packet

            packet.getData(generalBuffer, ROIConstants::ROIMAXPACKETPAYLOAD);  // Get the data from
                                                                               // the packet
            forwardPacket.setData(generalBuffer,
                                  ROIConstants::ROIMAXPACKETPAYLOAD);  // Set the data of the
                                                                       // forward packet

            chainManager.chainForward(
                forwardPacket);  // Forward the packet to the next module in the chain

#if DEBUG && defined(__AVR__)
            Serial.println(F("Sent PINGLOOPBACK"));
#endif
        }
    }

    uint16_t actionCode = packet.getActionCode();  // Get the action code from the packet

    ROIPackets::sysAdminPacket replyPacket = packet.swapReply();  // Create a reply packet

    switch (actionCode) {
        case sysAdminConstants::PING: {  // if responding to a ping

#if DEBUG && defined(__AVR__)
            Serial.println(F("Ping"));
#endif

            uint8_t pingResponse[2];
            pingResponse[0] = statusManager.getOperable();
            pingResponse[1] = moduleType;  // Return the module type (set on construction)
            replyPacket.setData(pingResponse, sizeof(pingResponse) / sizeof(pingResponse[0]));
            replyPacket.setActionCode(sysAdminConstants::PONG);  // Set the action code to PONG
            break;
        }

        case sysAdminConstants::STATUSREPORT: {  // if responding to a status report request

#if DEBUG && defined(__AVR__)
            Serial.println(F("Status Report"));
#endif

            uint8_t statusReport[14];
            statusReport[0] = statusManager.getSystemStatus();  // Get the system status

            statusReport[1] = millis() / 3600000;       // Hours since last reset
            statusReport[2] = (millis() / 60000) % 60;  // Minutes since last reset
            statusReport[3] = (millis() / 1000) % 60;   // Seconds since last reset

            uint16_t vcc = supplyVoltageReader::getAccurateVCC();  // Get the VCC
            statusReport[4] = highByte(vcc);
            statusReport[5] = lowByte(vcc);

            statusReport[6] = moduleType;  // Return the module type

            statusReport[7] =
                chainManager.getChainNeighborOctet();  // Return the chain neighbor octet

            for (int i = 0; i < 6; i++) {
                statusReport[i + 8] = mac[i];
            }

            replyPacket.setData(statusReport, sizeof(statusReport) / sizeof(statusReport[0]));
            break;
        }

        case sysAdminConstants::BLACKLIST: {   // if responding to a blacklist request
            Packet.getData(generalBuffer, 2);  // Set the data of the
            // reply packet

#if DEBUG && defined(__AVR__)
            Serial.println(F("Blacklist"));
#endif

            switch (generalBuffer[0]) {
                case blacklistConstants::ADDBLACKLIST: {  // if adding an octet to the blacklist
                    blacklistManager.addBlacklist(
                        generalBuffer[1]);  // Add the octet to the blacklist
                    generalBuffer[0] = 1;   // reuse generalBuffer buffer to send back a response
                    replyPacket.setData(generalBuffer, 1);  // Set the data of the reply packet
                    break;
                }

                case blacklistConstants::REMOVEBLACKLIST: {  // if removing an octet from the
                                                             // blacklist
                    blacklistManager.removeBlacklist(
                        generalBuffer[1]);  // Remove the octet from the blacklist
                    generalBuffer[0] = 1;   // reuse generalBuffer buffer to send back a response
                    replyPacket.setData(generalBuffer, 1);  // Set the data of the reply packet
                    break;
                }

                case blacklistConstants::LISTBLACKLIST: {  // if exporting the blacklist
                    blacklistManager.exportBlacklist(
                        generalBuffer,
                        ROIConstants::ROIMAXPACKETPAYLOAD);  // Export the blacklist to the buffer
                    replyPacket.setData(generalBuffer,
                                        ROIConstants::ROIMAXPACKETPAYLOAD);  // Set the data of the
                                                                             // reply packet
                    break;
                }

                default: {
                    replyPacket.setActionCode(
                        sysAdminConstants::BLANK);  // Set the action code to BLANK
                    break;  // Do nothing, ig we send a blank packet out. Sorry for the space
                            // junk.
                }
            }
        }

        default: {
            replyPacket.setActionCode(sysAdminConstants::BLANK);  // Set the action code to BLANK
            break;  // Do nothing, ig we send a blank packet out. Sorry for the space
                    // junk.
        }
    }
    return replyPacket;
}
