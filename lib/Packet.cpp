#include "Packet.h"
using namespace ROIPackets;
// General Packet

Packet::Packet(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
               uint16_t subDeviceID, uint16_t actionCode, uint8_t* data, uint16_t dataSize)
               : Packet(networkAddress, hostAddressOctet, clientAddressOctet,
                        subDeviceID, actionCode) {

    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD && i < dataSize; i++) {  // initialize data array
        this->data[i] = data[i];
    }
}

Packet::Packet(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
               uint16_t subDeviceID, uint16_t actionCode) {
    this->networkAddress = networkAddress;
    this->hostAddressOctet = hostAddressOctet;
    this->clientAddressOctet = clientAddressOctet;
    this->subDeviceID = subDeviceID;
    this->actionCode = actionCode;

    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {
        this->data[i] = 0;
    }
}

Packet::Packet(uint8_t hostAddressOctet, uint8_t clientAddressOctet) : Packet(0, hostAddressOctet, clientAddressOctet, 0, 0) {}

Packet::Packet() : Packet(0, 0, 0, 0, 0) {}

Packet::~Packet() {}



uint32_t Packet::getNetworkAddress() { return this->networkAddress; }

uint8_t Packet::getHostAddressOctet() { return this->hostAddressOctet; }

uint8_t Packet::getClientAddressOctet() { return this->clientAddressOctet; }

uint32_t Packet::getClientIP() { return this->networkAddress | this->clientAddressOctet; }

uint16_t Packet::getSubDeviceID() { return this->subDeviceID; }

uint16_t Packet::getActionCode() { return this->actionCode; }

void Packet::getData(uint8_t* dataBuffer, uint16_t dataBufferSize) {
    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD && i < dataBufferSize; i++) {
        dataBuffer[i] = this->data[i];
    }
}



void Packet::setNetworkAddress(uint32_t networkAddress) { this->networkAddress = networkAddress; }

void Packet::setHostAddressOctet(uint8_t hostAddressOctet) {
    this->hostAddressOctet = hostAddressOctet;
}

void Packet::setClientAddressOctet(uint8_t clientAddressOctet) {
    this->clientAddressOctet = clientAddressOctet;
}

void Packet::setSubDeviceID(uint16_t subDeviceID) { this->subDeviceID = subDeviceID; }

void Packet::setActionCode(uint16_t actionCode) { this->actionCode = actionCode; }

void Packet::setData(uint8_t* data, uint16_t dataSize) {
    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD && i < dataSize;
         i++) {  // initialize data array
        this->data[i] = data[i];
    }
}



bool Packet::importPacket(uint8_t* packet, uint16_t packetSize) {
    if (packetSize < 6) return false;  // packet is too small to be a Packet, even without payload

    this->subDeviceID = (packet[0] << 8) | packet[1];
    this->actionCode = (packet[2] << 8) | packet[3];
    this->checksum = (packet[4] << 8) | packet[5];
    for (uint16_t i = 6; i < ROIConstants::ROIMAXPACKETPAYLOAD + 6 && i < packetSize; i++) {
        this->data[i - 6] = packet[i];  // copy data into data array
    }
    return validateChecksum();  // validate checksum
}

bool Packet::exportPacket(uint8_t* packetBuffer, uint16_t packetBufferSize) {
    if (packetBufferSize < ROIConstants::ROIMAXPACKETPAYLOAD + 6)
        return false;  // packetBuffer is too small to hold the packet

    if (this->checksum == 0) {  // if checksum is not set, calculate it and set it
        this->checksum = calculateChecksum();
    }

    packetBuffer[0] = (this->subDeviceID >> 8) & 0xff;
    packetBuffer[1] = this->subDeviceID & 0xff;
    packetBuffer[2] = (this->actionCode >> 8) & 0xff;
    packetBuffer[3] = this->actionCode & 0xff;
    packetBuffer[4] = (this->checksum >> 8) & 0xff;
    packetBuffer[5] = this->checksum & 0xff;

    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD && i < packetBufferSize - 6; i++) {
        packetBuffer[i + 6] = this->data[i];
    }
    return true;
}

uint16_t Packet::calculateChecksum() {
    uint16_t checksum = 0;

    // Sum all of the bytes in the packet
    // It will probably overflow, but that's okay because it only needs to be deterministic,
    // the exact value doesn't matter. We will not be back-calculating the data from the checksum.
    // If the checksum is wrong, the packet will be discarded, and the request process will be
    // retried.
    checksum += this->subDeviceID;
    checksum += this->actionCode;
    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {
        checksum += this->data[i];
    }

    return checksum;
}

void Packet::setChecksum(uint16_t checksum) { this->checksum = checksum; }

bool Packet::validateChecksum() { return this->checksum == calculateChecksum(); }




/// sysAdminPacket

sysAdminPacket::sysAdminPacket(uint32_t networkAddress, uint8_t hostAddressOctet,
                               uint8_t originHostOctet, uint8_t clientAddressOctet,
                               uint16_t actionCode, uint8_t* data, uint16_t dataBufferSize,
                               uint16_t adminMetaData) 
                               : sysAdminPacket(networkAddress, hostAddressOctet,
                                                originHostOctet, clientAddressOctet,
                                                actionCode, adminMetaData) {
    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD && i < dataBufferSize;
         i++) {  // initialize data array
        this->data[i] = data[i];
    }
}

sysAdminPacket::sysAdminPacket(uint32_t networkAddress, uint8_t hostAddressOctet,
                               uint8_t originHostOctet, uint8_t clientAddressOctet,
                               uint16_t actionCode, uint16_t adminMetaData) {
    this->networkAddress = networkAddress;
    this->hostAddressOctet = hostAddressOctet;
    this->clientAddressOctet = clientAddressOctet;
    this->subDeviceID = subDeviceID;
    this->actionCode = actionCode;
    this->adminMetaData = adminMetaData;
    this->originHostOctet = originHostOctet;

    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {
        this->data[i] = 0;
    }
}

sysAdminPacket::sysAdminPacket(uint8_t hostAddressOctet, uint8_t clientAddressOctet) 
                               : sysAdminPacket(0, hostAddressOctet, hostAddressOctet,
                                                clientAddressOctet, 0, 0) {}

sysAdminPacket::sysAdminPacket() : sysAdminPacket(0, 0, 0, 0, 0, 0) {}

sysAdminPacket::~sysAdminPacket() {}



uint16_t sysAdminPacket::getAdminMetaData() { return this->adminMetaData; }

uint8_t sysAdminPacket::getOriginHostOctet() { return this->originHostOctet; }



void sysAdminPacket::setAdminMetaData(uint16_t adminMetaData) {
    this->adminMetaData = adminMetaData;
}

void sysAdminPacket::setOriginHostOctet(uint8_t originHostOctet) {
    this->originHostOctet = originHostOctet;
}



bool sysAdminPacket::importPacket(uint8_t* packet, uint16_t packetSize) {
    if (packetSize < 7) return false;  // packet is too small to be a sysAdminPacket
    /* Layout of sysAdminPacket:
    admin metadata uint16
    originator host address uint8
    action code uint16
    data uint8-255
    */

    this->adminMetaData = (packet[0] << 8) | packet[1];
    this->originHostOctet = packet[2];
    this->actionCode = (packet[3] << 8) | packet[4];
    this->checksum = (packet[5] << 8) | packet[6];
    for (uint16_t i = 7; i < ROIConstants::ROIMAXPACKETPAYLOAD + 7 && i < packetSize;
         i++) {  // initialize data array
        this->data[i - 7] = packet[i];
    }
    return validateChecksum();  // validate checksum
}

bool sysAdminPacket::exportPacket(uint8_t* packetBuffer, uint16_t packetBufferSize) {
    if (packetBufferSize < ROIConstants::ROIMAXPACKETPAYLOAD + 5)
        return false;  // packetBuffer is too small
    /* Layout of sysAdminPacket:
    admin metadata uint16
    originator host address uint8
    action code uint16
    data uint8-255
    */

    if (this->checksum == 0) {  // if checksum is not set, calculate it and set it
        this->checksum = calculateChecksum();
    }

    packetBuffer[0] = (this->adminMetaData >> 8) & 0xff;
    packetBuffer[1] = this->adminMetaData & 0xff;
    packetBuffer[2] = this->originHostOctet;
    packetBuffer[3] = (this->actionCode >> 8) & 0xff;
    packetBuffer[4] = this->actionCode & 0xff;
    packetBuffer[5] = (this->checksum >> 8) & 0xff;
    packetBuffer[6] = this->checksum & 0xff;
    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD && i < packetBufferSize - 7; i++) {
        packetBuffer[i + 7] = this->data[i];
    }
    return true;
}

uint16_t sysAdminPacket::calculateChecksum() {
    uint16_t checksum = 0;

    // Sum all of the bytes in the packet
    // It will probably overflow, but that's okay because it only needs to be deterministic,
    // the exact value doesn't matter. We will not be back-calculating the data from the checksum.
    // If the checksum is wrong, the packet will be discarded, and the request process will be
    // retried.
    checksum += this->adminMetaData;
    checksum += this->originHostOctet;
    checksum += this->actionCode;
    for (uint16_t i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {
        checksum += this->data[i];
    }

    return checksum;
}
