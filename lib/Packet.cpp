#import "Packet.h"

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