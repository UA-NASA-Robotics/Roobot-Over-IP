#ifndef PACKETS_H
#define PACKETS_H

// #include <vector> //Not implemented in Arduino, plus we need to be more memory efficient
#include <stdint.h>
namespace ROIConstants {
// Constants for the ROI module

constexpr uint16_t ROIGENERALPORT =
    57344;  // The port that the ROI module listens on for general
            // packets. This is the first port in the range 2^16 - 2^13
constexpr uint16_t ROIINTERUPTPORT =
    57600;  // The port that the ROI module listens on for interrupt
            // packets. This is the second port in the range
constexpr uint16_t ROISYSADMINPORT = 57664;  // The port that the ROI module listens on for sysAdmin
                                             // packets. This is the third port in the range

/*
The Ethernet Library has a defined value "UDP_TX_PACKET_MAX_SIZE" which is the maximum size of a UDP
The value given is actually arbitrary, with no real reason for the value. The value is 24 bytes.

The W5500 datasheet states that the maximum size of a packet can fill the 16KB buffer.
However, it must be manually checked if the buffer can accept a packet.
*/

constexpr uint16_t ROIMAXPACKETSIZE = 60;  // The maximum size of a packet
constexpr uint16_t ROIMAXPACKETPAYLOAD =
    52;  // The maximum size of the payload of a packet (ROIMAXPACKETSIZE - 8)

}  // namespace ROIConstants

namespace ROIPackets {
class Packet {
   protected:
    uint32_t networkAddress;     // 4 bytes IP address
    uint8_t hostAddressOctet;    // 1 byte host address octet
    uint8_t clientAddressOctet;  // 1 byte client address octet

    uint16_t subDeviceID;                             // 2 bytes subdevice ID
    uint16_t actionCode;                              // 2 bytes action code
    uint8_t data[ROIConstants::ROIMAXPACKETPAYLOAD];  // 0-x bytes of data
    uint16_t checksum;                                // 2 bytes checksum

   public:
    // Constructor
    Packet(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
           uint16_t subDeviceID, uint16_t actionCode, uint8_t *data, uint16_t dataSize);
    Packet(uint8_t hostAddressOctet, uint8_t clientAddressOctet);
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
    void getData(uint8_t *dataBuffer, uint16_t dataBufferSize);

    // Setters
    void setNetworkAddress(uint32_t networkAddress);
    void setHostAddressOctet(uint8_t hostAddressOctet);
    void setClientAddressOctet(uint8_t clientAddressOctet);

    void setSubDeviceID(uint16_t subDeviceID);
    void setActionCode(uint16_t actionCode);
    void setData(uint8_t *data, uint16_t dataSize);

    // IO
    bool importPacket(uint8_t *packet, uint16_t packetSize);
    bool exportPacket(uint8_t *packetBuffer, uint16_t packetBufferSize);

    // Checksum

    uint16_t calculateChecksum();         // Calculate the checksum of the packet
    void setChecksum(uint16_t checksum);  // Set the checksum of the packet

    bool validateChecksum();  // Validate the checksum of the packet matches the calculated checksum
};

class sysAdminPacket : public Packet {
   protected:
    uint16_t adminMetaData;  // extra data for sysadmin packets providing additional information

   public:
    // Constructor
    sysAdminPacket(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
                   uint16_t actionCode, uint8_t *data, uint16_t dataBufferSize,
                   uint16_t adminMetaData);
    sysAdminPacket(uint8_t hostAddressOctet, uint8_t clientAddressOctet);
    sysAdminPacket();

    // Destructor
    ~sysAdminPacket();

    // Getters
    uint16_t getAdminMetaData();

    // Setters
    void setAdminMetaData(uint16_t adminMetaData);

    // IO
    bool importPacket(uint8_t *packet, uint16_t packetSize);
    bool exportPacket(uint8_t *packetBuffer, uint16_t packetBufferSize);

    // Checksum
    uint16_t
    calculateChecksum();  // Calculate the checksum of the packet, including the adminMetaData
                          // Validate the checksum function is inherited from the Packet class
};

}  // namespace ROIPackets
#endif  // PACKETS_H