#include "Packet.h"
using namespace ROIPackets;
// General Packet

Packet::Packet(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
               uint16_t subDeviceID, uint16_t actionCode, uint8_t* data) {
    this->networkAddress = networkAddress;
    this->hostAddressOctet = hostAddressOctet;
    this->clientAddressOctet = clientAddressOctet;
    this->subDeviceID = subDeviceID;
    this->actionCode = actionCode;
    for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {  // initialize data array
        this->data[i] = data[i];
    }
}

Packet::Packet() {
    this->networkAddress = 0;
    this->hostAddressOctet = 0;
    this->clientAddressOctet = 0;
    this->subDeviceID = 0;
    this->actionCode = 0;
    for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {  // initialize data array
        this->data[i] = 0;
    }
}

Packet::~Packet() {}

uint32_t Packet::getNetworkAddress() { return this->networkAddress; }

uint8_t Packet::getHostAddressOctet() { return this->hostAddressOctet; }

uint8_t Packet::getClientAddressOctet() { return this->clientAddressOctet; }

uint32_t Packet::getClientIP() { return this->networkAddress | this->clientAddressOctet; }

uint16_t Packet::getSubDeviceID() { return this->subDeviceID; }

uint16_t Packet::getActionCode() { return this->actionCode; }

void Packet::getData(uint8_t* dataBuffer) {
    for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {
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

void Packet::setData(uint8_t* data) {
    for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {  // initialize data array
        this->data[i] = data[i];
    }
}

bool Packet::importPacket(uint8_t* packet) {
    this->subDeviceID = (packet[0] << 8) | packet[1];
    this->actionCode = (packet[2] << 8) | packet[3];
    for (int i = 4; i < ROIConstants::ROIMAXPACKETPAYLOAD + 4; i++) {
        this->data[i - 4] = packet[i];  // copy data into data array
    }
}

bool Packet::exportPacket(uint8_t* packetBuffer) {
    if (sizeof(packetBuffer) < ROIConstants::ROIMAXPACKETPAYLOAD + 4)
        return false;  // packetBuffer is too small
    packetBuffer[0] = (this->subDeviceID >> 8) & 0xff;
    packetBuffer[1] = this->subDeviceID & 0xff;
    packetBuffer[2] = (this->actionCode >> 8) & 0xff;
    packetBuffer[3] = this->actionCode & 0xff;
    for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {
        packetBuffer[i + 4] = this->data[i];
    }
    return true;
}

/// sysAdminPacket

sysAdminPacket::sysAdminPacket(uint32_t networkAddress, uint8_t hostAddressOctet,
                               uint8_t clientAddressOctet, uint16_t actionCode, uint8_t* data,
                               uint16_t adminMetaData) {
    this->networkAddress = networkAddress;
    this->hostAddressOctet = hostAddressOctet;
    this->clientAddressOctet = clientAddressOctet;
    this->subDeviceID = subDeviceID;
    this->actionCode = actionCode;
    for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {  // initialize data array
        this->data[i] = data[i];
    }
    this->adminMetaData = adminMetaData;
}

sysAdminPacket::sysAdminPacket() {
    this->networkAddress = 0;
    this->hostAddressOctet = 0;
    this->clientAddressOctet = 0;
    this->subDeviceID = 0;
    this->actionCode = 0;
    for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {  // initialize data array
        this->data[i] = 0;
    }
    this->adminMetaData = 0;
}

sysAdminPacket::~sysAdminPacket() {}

uint16_t sysAdminPacket::getAdminMetaData() { return this->adminMetaData; }

void sysAdminPacket::setAdminMetaData(uint16_t adminMetaData) {
    this->adminMetaData = adminMetaData;
}

bool sysAdminPacket::importPacket(uint8_t* packet) {
    /* Layout of sysAdminPacket:
    admin metadata uint16
    originator host address uint8
    action code uint16
    data uint8-255
    */

    this->adminMetaData = (packet[0] << 8) | packet[1];
    this->hostAddressOctet = packet[2];
    this->actionCode = (packet[3] << 8) | packet[4];
    for (int i = 5; i < ROIConstants::ROIMAXPACKETPAYLOAD + 4; i++) {  // initialize data array
        this->data[i - 5] = packet[i];
    }
    return true;
}

bool sysAdminPacket::exportPacket(uint8_t* packetBuffer) {
    if (sizeof(packetBuffer) < ROIConstants::ROIMAXPACKETPAYLOAD + 5)
        return false;  // packetBuffer is too small
    /* Layout of sysAdminPacket:
    admin metadata uint16
    originator host address uint8
    action code uint16
    data uint8-255
    */

    packetBuffer[0] = (this->adminMetaData >> 8) & 0xff;
    packetBuffer[1] = this->adminMetaData & 0xff;
    packetBuffer[2] = this->hostAddressOctet;
    packetBuffer[3] = (this->actionCode >> 8) & 0xff;
    packetBuffer[4] = this->actionCode & 0xff;
    for (int i = 0; i < ROIConstants::ROIMAXPACKETPAYLOAD; i++) {
        packetBuffer[i + 5] = this->data[i];
    }
    return true;
}
