#import "Packet.h"

// General Packet

Packet::Packet(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet) {
    this->networkAddress = networkAddress;
    this->hostAddressOctet = hostAddressOctet;
    this->clientAddressOctet = clientAddressOctet;
    this->subDeviceID = 0;
    this->actionCode = 0;
    this->data = std::vector<uint8_t>();
}

Packet::Packet(uint32_t IPaddress, uint8_t clientAddressOctet) {
    this->networkAddress = IPaddress & 0xFFFFFF00;
    this->hostAddressOctet = IPaddress & 0x000000FF;
    this->clientAddressOctet = clientAddressOctet;
    this->subDeviceID = 0;
    this->actionCode = 0;
    this->data = std::vector<uint8_t>();
}

Packet::Packet(uint16_t subDeviceID, uint16_t actionCode, std::vector<uint8_t> data) {
    this->networkAddress = 0;
    this->hostAddressOctet = 0;
    this->clientAddressOctet = 0;
    this->subDeviceID = subDeviceID;
    this->actionCode = actionCode;
    this->data = data;
}

Packet::Packet() {
    this->networkAddress = 0;
    this->hostAddressOctet = 0;
    this->clientAddressOctet = 0;
    this->subDeviceID = 0;
    this->actionCode = 0;
    this->data = std::vector<uint8_t>();
}

Packet::~Packet() { this->data.clear(); }

uint32_t Packet::getNetworkAddress() { return this->networkAddress; }

uint8_t Packet::getHostAddressOctet() { return this->hostAddressOctet; }

uint8_t Packet::getClientAddressOctet() { return this->clientAddressOctet; }

uint32_t Packet::getClientIP() { return this->networkAddress | this->clientAddressOctet; }

uint16_t Packet::getSubDeviceID() { return this->subDeviceID; }

uint16_t Packet::getActionCode() { return this->actionCode; }

std::vector<uint8_t> Packet::getData() { return this->data; }

void Packet::setNetworkAddress(uint32_t networkAddress) { this->networkAddress = networkAddress; }

void Packet::setHostAddressOctet(uint8_t hostAddressOctet) {
    this->hostAddressOctet = hostAddressOctet;
}

void Packet::setClientAddressOctet(uint8_t clientAddressOctet) {
    this->clientAddressOctet = clientAddressOctet;
}

void Packet::setSubDeviceID(uint16_t subDeviceID) { this->subDeviceID = subDeviceID; }

void Packet::setActionCode(uint16_t actionCode) { this->actionCode = actionCode; }

void Packet::setData(std::vector<uint8_t> data) { this->data = data; }

bool Packet::importPacket(std::vector<uint8_t> packet) {
    if (packet.size() < 5) {  // 5 bytes minimum with 1 data byte
        return false;
    }

    this->subDeviceID = (packet[0] << 8) | packet[1];
    this->actionCode = (packet[2] << 8) | packet[3];
    this->data.clear();
    for (int i = 4; i < packet.size(); i++) {
        this->data.push_back(packet[i]);
    }
}

std::vector<uint8_t> Packet::exportPacket() {
    std::vector<uint8_t> packet;
    packet.push_back((this->subDeviceID >> 8) & 0xFF);
    packet.push_back(this->subDeviceID & 0xFF);
    packet.push_back((this->actionCode >> 8) & 0xFF);
    packet.push_back(this->actionCode & 0xFF);
    for (int i = 0; i < this->data.size(); i++) {
        packet.push_back(this->data[i]);
    }
    return packet;
}

/// sysAdminPacket

sysAdminPacket::sysAdminPacket(uint32_t networkAddress, uint8_t hostAddressOctet,
                               uint8_t clientAddressOctet) {
    this->networkAddress = networkAddress;
    this->hostAddressOctet = hostAddressOctet;
    this->clientAddressOctet = clientAddressOctet;
    this->subDeviceID = 0;
    this->actionCode = 0;
    this->data = std::vector<uint8_t>();
}

sysAdminPacket::sysAdminPacket(uint32_t HostIPaddress, uint8_t clientAddressOctet,
                               uint16_t subDeviceID, uint16_t actionCode, std::vector<uint8_t> data,
                               uint16_t adminMetaData) {
    this->networkAddress = IPaddress & 0xFFFFFF00;
    this->hostAddressOctet = IPaddress & 0x000000FF;
    this->clientAddressOctet = clientAddressOctet;
    this->subDeviceID = subDeviceID;
    this->actionCode = actionCode;
    this->data = data;
    this->adminMetaData = adminMetaData;
}

sysAdminPacket::sysAdminPacket(uint16_t adminMetaData, uint16_t subDeviceID, uint16_t actionCode,
                               std::vector<uint8_t> data) {
    this->networkAddress = 0;
    this->hostAddressOctet = 0;
    this->clientAddressOctet = 0;
    this->subDeviceID = subDeviceID;
    this->actionCode = actionCode;
    this->data = data;
    this->adminMetaData = adminMetaData;
}

sysAdminPacket::sysAdminPacket() {
    this->networkAddress = 0;
    this->hostAddressOctet = 0;
    this->clientAddressOctet = 0;
    this->subDeviceID = 0;
    this->actionCode = 0;
    this->data = std::vector<uint8_t>();
    this->adminMetaData = 0;
}

sysAdminPacket::~sysAdminPacket() { this->data.clear(); }

uint16_t sysAdminPacket::getAdminMetaData() { return this->adminMetaData; }

void sysAdminPacket::setAdminMetaData(uint16_t adminMetaData) {
    this->adminMetaData = adminMetaData;
}

bool sysAdminPacket::importPacket(std::vector<uint8_t> packet) {
    if (packet.size() < 6) {  // 6 bytes minimum with 1 data byte
        return false;
    }
    /* Layout of sysAdminPacket:
    admin metadata uint16
    originator host address uint8
    action code uint16
    data uint8-255
    */

    this->adminMetaData = (packet[0] << 8) | packet[1];
    this->hostAddressOctet = packet[2];
    this->actionCode = (packet[3] << 8) | packet[4];
    this->data.clear();
    for (int i = 5; i < packet.size(); i++) {
        this->data.push_back(packet[i]);
    }
    return true;
}

std::vector<uint8_t> sysAdminPacket::exportPacket() {
    std::vector<uint8_t> packet;
    packet.push_back((this->adminMetaData >> 8) & 0xFF);
    packet.push_back(this->adminMetaData & 0xFF);
    packet.push_back(this->hostAddressOctet);
    packet.push_back((this->actionCode >> 8) & 0xFF);
    packet.push_back(this->actionCode & 0xFF);
    for (int i = 0; i < this->data.size(); i++) {
        packet.push_back(this->data[i]);
    }
    return packet;
}

void sysAdminPacket::printPacket() {
    std::cout << "Admin Metadata: " << this->adminMetaData << std::endl;
    std::cout << "Host Address: " << this->hostAddressOctet << std::endl;
    std::cout << "Action Code: " << this->actionCode << std::endl;
    std::cout << "Data: ";
    for (int i = 0; i < this->data.size(); i++) {
        std::cout << this->data[i] << " ";
    }
    std::cout << std::endl;
}
