#include "sysAdminHandler.h"

sysAdminHandler::sysAdminHandler(uint8_t moduleType, statusManager::statusManager statusManager,
                                 chainNeighborManager::chainNeighborManager chainManager) {
    macHelper = macGen::macAddressHelper();
    macHelper.getMacAddress(mac);
    this->moduleType = moduleType;
    this->statusManager = statusManager;
    this->chainManager = chainManager;
}

sysAdminHandler::~sysAdminHandler() {}

ROIPackets::sysAdminPacket sysAdminHandler::handleSysAdminPacket(
    ROIPackets::sysAdminPacket packet) {
    uint16_t metaData = packet.getAdminMetaData();  // Get the metacode from the packet
    bool chainedMessage = metaData & sysAdminConstants::CHAINMESSAGEMETA;
    metaData &= ~sysAdminConstants::CHAINMESSAGEMETA;  // Remove the chain message metadata for
                                                       // the response

    uint8_t replyHostOctet = packet.getHostAddressOctet();  // Get the host octet from the packet
                                                            // (This is the default reply address)

    // Handle forwarding chain messages
    if (chainedMessage) {
        replyHostOctet = packet.getAdminMetaData() &
                         0xFF;  // Get the the reply host octet from the metadata if it is a chained
                                // message, essentially overloads the reply destination

        if (packet.getOriginHostOctet() != chainManager.getChainNeighborOctet() &&
            chainManager.chainNeighborConnected()) {  // if this the next module in the chain is the
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
            forwardPacket.setData(
                packet.getData(),
                ROIConstants::ROIMAXPACKETPAYLOAD);  // Set the data of the forward packet

            chainManager.chainForward(
                forwardPacket);  // Forward the packet to the next module in the chain
        }
    }

    uint16_t actionCode = packet.getActionCode();  // Get the action code from the packet

    ROIPackets::sysAdminPacket replyPacket;  // Create a reply packet
    replyPacket.setNetworkAddress(packet.getNetworkAddress());
    replyPacket.setClientAddressOctet(replyHostOctet);  // We were the client as the recipient of
                                                        // the packet, now we
    // are the host
    replyPacket.setHostAddressOctet(
        packet.getClientAddressOctet());  // We are the host swapping the client address

    replyPacket.setAdminMetaData(metaData);  // Set the metadata of the reply packet
    replyPacket.setActionCode(actionCode);   // Set the action code of the reply packet

    switch (actionCode) {
        case sysAdminConstants::PING:  // if responding to a ping
            uint8_t pingResponse[2];
            pingResponse[0] = statusManager.operable();
            pingResponse[1] = moduleType;  // Return the module type (set on construction)
            replyPacket.setData(pingResponse, sizeof(pingResponse) / sizeof(pingResponse[0]));
            replyPacket.setActionCode(sysAdminConstants::PONG);  // Set the action code to PONG
            break;

        case sysAdminConstants::STATUSREPORT:  // if responding to a status report request
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
                chainNeighbor ? chainNeighbor : 0;  // Return the chain neighbor if it exists

            for (int i = 0; i < 6; i++) {
                statusReport[i + 8] = mac[i];
            }

            replyPacket.setData(statusReport, sizeof(statusReport) / sizeof(statusReport[0]));
            break;
    }
    return replyPacket;
}
