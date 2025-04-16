#include "Packet.h"
using namespace ROIPackets;
// General Packet

Packet::Packet(uint8_t hostAddressOctet, uint8_t clientAddressOctet, uint16_t subDeviceID,
               uint16_t actionCode, uint8_t* data, uint16_t dataSize)
    : Packet(hostAddressOctet, clientAddressOctet, subDeviceID, actionCode) {
    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD && i < dataSize;
         i++) {  // initialize data array
        this->_data[i] = data[i];
    }
}

Packet::Packet(uint8_t hostAddressOctet, uint8_t clientAddressOctet, uint16_t subDeviceID,
               uint16_t actionCode) {
    this->_hostAddressOctet = hostAddressOctet;
    this->_clientAddressOctet = clientAddressOctet;
    this->_subDeviceID = subDeviceID;
    this->_actionCode = actionCode;

    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD; i++) {
        this->_data[i] = 0;
    }
}

Packet::Packet(uint8_t hostAddressOctet, uint8_t clientAddressOctet)
    : Packet(hostAddressOctet, clientAddressOctet, 0, 0) {}

Packet::Packet() : Packet(0, 0, 0, 0) {}

Packet::~Packet() {}

uint8_t Packet::getHostAddressOctet() { return this->_hostAddressOctet; }

uint8_t Packet::getClientAddressOctet() { return this->_clientAddressOctet; }

uint16_t Packet::getSubDeviceID() { return this->_subDeviceID; }

uint16_t Packet::getActionCode() { return this->_actionCode; }

void Packet::getData(uint8_t* dataBuffer, uint16_t dataBufferSize) {
    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD && i < dataBufferSize; i++) {
        dataBuffer[i] = this->_data[i];
    }
}

uint8_t& Packet::operator[](uint16_t index) { return this->_data[index]; }

void Packet::setHostAddressOctet(uint8_t hostAddressOctet) {
    this->_hostAddressOctet = hostAddressOctet;
}

void Packet::setClientAddressOctet(uint8_t clientAddressOctet) {
    this->_clientAddressOctet = clientAddressOctet;
}

void Packet::setSubDeviceID(uint16_t subDeviceID) { this->_subDeviceID = subDeviceID; }

void Packet::setActionCode(uint16_t actionCode) { this->_actionCode = actionCode; }

void Packet::setData(uint8_t* data, uint16_t dataSize) {
    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD && i < dataSize;
         i++) {  // initialize data array
        this->_data[i] = data[i];
    }
}
void Packet::setData(uint8_t num1) { this->_data[0] = num1; }
void Packet::setData(uint8_t num1, uint8_t num2) {
    this->_data[0] = num1;
    this->_data[1] = num2;
}
void Packet::setData(uint8_t num1, uint8_t num2, uint8_t num3, uint8_t num4) {
    this->_data[0] = num1;
    this->_data[1] = num2;
    this->_data[2] = num3;
    this->_data[3] = num4;
}

void Packet::setData(float num1) { floatCast::floatToUint8Array(num1, this->_data, 3, 0); }

void Packet::setData(uint16_t num1) { this-> _data[0] = num1>>8; this->_data[1] = num1 & 0xFF;}

void Packet::setData(float num1, bool endian) {
    if (endian) {
        floatCast::floatToUint8Array(num1, this->_data, 3, 0);
    } else {
        floatCast::floatToUint8Array(num1, this->_data, 0, 3);
    }
}

bool Packet::importPacket(uint8_t* packet, uint16_t packetSize) {
    if (packetSize < 6) return false;  // packet is too small to be a Packet, even without payload

    this->_subDeviceID = (packet[0] << 8) | packet[1];
    this->_actionCode = (packet[2] << 8) | packet[3];
    this->_checksum = (packet[4] << 8) | packet[5];
    for (uint16_t i = 6; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD + 6 && i < packetSize; i++) {
        this->_data[i - 6] = packet[i];  // copy data into data array
    }
    return validateChecksum();  // validate checksum
}

bool Packet::exportPacket(uint8_t* packetBuffer, uint16_t packetBufferSize) {
    if (packetBufferSize < ROIConstants::ROI_MAX_PACKET_PAYLOAD + 6)
        return false;  // packetBuffer is too small to hold the packet

    this->_checksum = calculateChecksum();  // calculate checksum

    packetBuffer[0] = (this->_subDeviceID >> 8) & 0xff;
    packetBuffer[1] = this->_subDeviceID & 0xff;
    packetBuffer[2] = (this->_actionCode >> 8) & 0xff;
    packetBuffer[3] = this->_actionCode & 0xff;
    packetBuffer[4] = (this->_checksum >> 8) & 0xff;
    packetBuffer[5] = this->_checksum & 0xff;

    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD && i < packetBufferSize - 6;
         i++) {
        packetBuffer[i + 6] = this->_data[i];
    }
    return true;
}

Packet Packet::swapReply() {
    Packet replyPacket =
        Packet(this->_clientAddressOctet, this->_hostAddressOctet, this->_subDeviceID,
               this->_actionCode);  // create a new packet with the host and client
                                    // addresses swapped, and empty payload
    return replyPacket;
}

uint16_t Packet::calculateChecksum() {
    uint16_t checksum = 0;

    // Sum all of the bytes in the packet
    // It will probably overflow, but that's okay because it only needs to be deterministic,
    // the exact value doesn't matter. We will not be back-calculating the data from the checksum.
    // If the checksum is wrong, the packet will be discarded, and the request process will be
    // retried.
    checksum += this->_subDeviceID;
    checksum += this->_actionCode;
    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD; i++) {
        checksum += this->_data[i];
    }

    return checksum;
}

void Packet::setChecksum(uint16_t checksum) { this->_checksum = checksum; }

bool Packet::validateChecksum() { return this->_checksum == calculateChecksum(); }

/// sysAdminPacket

sysAdminPacket::sysAdminPacket(uint8_t hostAddressOctet, uint8_t originHostOctet,
                               uint8_t clientAddressOctet, uint16_t actionCode, uint8_t* data,
                               uint16_t dataBufferSize, uint16_t adminMetaData)
    : sysAdminPacket(hostAddressOctet, originHostOctet, clientAddressOctet, actionCode,
                     adminMetaData) {
    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD && i < dataBufferSize;
         i++) {  // initialize data array
        this->_data[i] = data[i];
    }
}

sysAdminPacket::sysAdminPacket(uint8_t hostAddressOctet, uint8_t originHostOctet,
                               uint8_t clientAddressOctet, uint16_t actionCode,
                               uint16_t adminMetaData) {
    this->_hostAddressOctet = hostAddressOctet;
    this->_clientAddressOctet = clientAddressOctet;
    this->_subDeviceID = _subDeviceID;
    this->_actionCode = actionCode;
    this->_adminMetaData = adminMetaData;
    this->_originHostOctet = originHostOctet;

    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD; i++) {
        this->_data[i] = 0;
    }
}

sysAdminPacket::sysAdminPacket(uint8_t hostAddressOctet, uint8_t clientAddressOctet)
    : sysAdminPacket(hostAddressOctet, hostAddressOctet, clientAddressOctet, 0, 0) {}

sysAdminPacket::sysAdminPacket() : sysAdminPacket(0, 0, 0, 0, 0) {}

sysAdminPacket::~sysAdminPacket() {}

uint16_t sysAdminPacket::getAdminMetaData() { return this->_adminMetaData; }

uint8_t sysAdminPacket::getOriginHostOctet() { return this->_originHostOctet; }

void sysAdminPacket::setAdminMetaData(uint16_t adminMetaData) {
    this->_adminMetaData = adminMetaData;
}

void sysAdminPacket::setOriginHostOctet(uint8_t originHostOctet) {
    this->_originHostOctet = originHostOctet;
}

bool sysAdminPacket::importPacket(uint8_t* packet, uint16_t packetSize) {
    if (packetSize < 7) return false;  // packet is too small to be a sysAdminPacket
    /* Layout of sysAdminPacket:
    admin metadata uint16
    originator host address uint8
    action code uint16
    data uint8-255
    */

    this->_adminMetaData = (packet[0] << 8) | packet[1];
    this->_originHostOctet = packet[2];
    this->_actionCode = (packet[3] << 8) | packet[4];
    this->_checksum = (packet[5] << 8) | packet[6];
    for (uint16_t i = 7; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD + 7 && i < packetSize;
         i++) {  // initialize data array
        this->_data[i - 7] = packet[i];
    }
    return validateChecksum();  // validate checksum
}

bool sysAdminPacket::exportPacket(uint8_t* packetBuffer, uint16_t packetBufferSize) {
    if (packetBufferSize < ROIConstants::ROI_MAX_PACKET_PAYLOAD + 5)
        return false;  // packetBuffer is too small
    /* Layout of sysAdminPacket:
    admin metadata uint16
    originator host address uint8
    action code uint16
    data uint8-255
    */

    if (this->_checksum == 0) {  // if checksum is not set, calculate it and set it
        this->_checksum = calculateChecksum();
    }

    packetBuffer[0] = (this->_adminMetaData >> 8) & 0xff;
    packetBuffer[1] = this->_adminMetaData & 0xff;
    packetBuffer[2] = this->_originHostOctet;
    packetBuffer[3] = (this->_actionCode >> 8) & 0xff;
    packetBuffer[4] = this->_actionCode & 0xff;
    packetBuffer[5] = (this->_checksum >> 8) & 0xff;
    packetBuffer[6] = this->_checksum & 0xff;
    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD && i < packetBufferSize - 7;
         i++) {
        packetBuffer[i + 7] = this->_data[i];
    }
    return true;
}

sysAdminPacket sysAdminPacket::swapReply() {
    sysAdminPacket replyPacket =
        sysAdminPacket(this->_clientAddressOctet, this->_originHostOctet, this->_hostAddressOctet,
                       this->_actionCode,
                       this->_adminMetaData);  // create a new packet with the
                                               // host and client addresses
                                               // swapped, and empty payload
    return replyPacket;
}

uint16_t sysAdminPacket::calculateChecksum() {
    uint16_t checksum = 0;

    // Sum all of the bytes in the packet
    // It will probably overflow, but that's okay because it only needs to be deterministic,
    // the exact value doesn't matter. We will not be back-calculating the data from the checksum.
    // If the checksum is wrong, the packet will be discarded, and the request process will be
    // retried.
    checksum += this->_adminMetaData;
    checksum += this->_originHostOctet;
    checksum += this->_actionCode;
    for (uint16_t i = 0; i < ROIConstants::ROI_MAX_PACKET_PAYLOAD; i++) {
        checksum += this->_data[i];
    }

    return checksum;
}
