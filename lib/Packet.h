#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

#include <iostream>
#include <vector>

#include "UnityTypes.h"

namespace ROIPackets {

class Packet {
   private:
    uint32_t networkAddress;     // 4 bytes IP address
    uint8_t hostAddressOctet;    // 1 byte host address
    uint8_t clientAddressOctet;  // 1 byte client address

    uint16_t subDeviceID;       // 2 bytes subdevice ID
    uint16_t actionCode;        // 2 bytes action code
    std::vector<uint8_t> data;  // 0-255 bytes data

   public:
    // Constructor
    Packet(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
           uint16_t subDeviceID, uint16_t actionCode, std::vector<uint8_t> data);
    Packet(uint32_t IPaddress, uint8_t clientAddressOctet, uint16_t subDeviceID,
           uint16_t actionCode, std::vector<uint8_t> data);
    Packet(uint16_t subDeviceID, uint16_t actionCode, std::vector<uint8_t> data);
    Packet();

    // Destructor
    ~Packet();

    // Getters
    uint32_t getNetworkAddress();
    uint8_t getHostAddressOctet();
    uint8_t getClientAddressOctet();

    uint32_t getClientIP();

    uint16_t getSubDeviceID();
    uint16_t getActionCode();
    std::vector<uint8_t> getData();

    // Setters
    void setNetworkAddress(uint32_t networkAddress);
    void setHostAddressOctet(uint8_t hostAddressOctet);
    void setClientAddressOctet(uint8_t clientAddressOctet);

    void setSubDeviceID(uint16_t subDeviceID);
    void setActionCode(uint16_t actionCode);
    void setData(std::vector<uint8_t> data);

    // IO
    bool importPacket(std::vector<uint8_t> packet);
    std::vector<uint8_t> exportPacket();

    // Print
    void printPacket();
};

class ChainNetworkPacket : public Packet {};

}  // namespace ROIPackets
#endif  // PACKETS_H