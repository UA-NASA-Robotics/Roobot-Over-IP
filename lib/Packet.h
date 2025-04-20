#ifndef PACKETS_H
#define PACKETS_H

// #include <vector> //Not implemented in Arduino, plus we need to be more memory efficient
#include <stdint.h>

#include "floatCast.h"  // For float to uint8_t conversion
namespace ROIConstants {
// Constants for the ROI module

constexpr uint16_t ROI_GENERAL_PORT = 57344;  // The port that the ROI module listens on for general
// packets. This is the first port in the range 2^16 - 2^13
constexpr uint16_t ROI_STREAM_PORT =
    57600;  // The port that the ROI module listens on for interrupt
// packets. This is the second port in the range
constexpr uint16_t ROI_SYS_ADMIN_PORT =
    57664;  // The port that the ROI module listens on for sysAdmin
// packets. This is the third port in the range

/*
The Ethernet Library has a defined value "UDP_TX_PACKET_MAX_SIZE" which is the maximum size of a UDP
The value given is actually arbitrary, with no real reason for the value. The value is 24 bytes.

The W5500 datasheet states that the maximum size of a packet can fill the 16KB buffer.
However, it must be manually checked if the buffer can accept a packet.
*/

/**
 * @brief The maximum size of a packet, this may be adjusted to
 *
 */
constexpr uint16_t ROI_MAX_PACKET_SIZE = 64;  // The maximum size of a packet
constexpr uint16_t ROI_MAX_PACKET_PAYLOAD =
    ROI_MAX_PACKET_SIZE -
    8;  // The maximum size of the payload of a packet (ROI_MAX_PACKET_SIZE - 8)

}  // namespace ROIConstants

namespace ROIPackets {
class Packet {
   protected:
    uint8_t _hostAddressOctet;    // 1 byte host address octet
    uint8_t _clientAddressOctet;  // 1 byte client address octet

    uint16_t _subDeviceID;                                // 2 bytes subdevice ID
    uint16_t _actionCode;                                 // 2 bytes action code
    uint8_t _data[ROIConstants::ROI_MAX_PACKET_PAYLOAD];  // 0-x bytes of data
    uint16_t _checksum;                                   // 2 bytes checksum

   public:
    // Constructor
    Packet(uint8_t hostAddressOctet, uint8_t clientAddressOctet, uint16_t subDeviceID,
           uint16_t actionCode, uint8_t* data, uint16_t dataSize);
    Packet(uint8_t hostAddressOctet, uint8_t clientAddressOctet, uint16_t subDeviceID,
           uint16_t actionCode);
    Packet(uint8_t hostAddressOctet, uint8_t clientAddressOctet);
    Packet();

    // Destructor
    ~Packet();

    // Getters
    uint8_t getHostAddressOctet();
    uint8_t getClientAddressOctet();

    uint16_t getSubDeviceID();
    uint16_t getActionCode();
    void getData(uint8_t* dataBuffer, uint16_t dataBufferSize);

    uint8_t& operator[](
        uint16_t index);  // Overload the [] operator to access the data array (read/write)

    // Setters
    void setHostAddressOctet(uint8_t hostAddressOctet);
    void setClientAddressOctet(uint8_t clientAddressOctet);

    void setSubDeviceID(uint16_t subDeviceID);
    void setActionCode(uint16_t actionCode);
    void setData(uint8_t* data, uint16_t dataSize);
    void setData(uint8_t num1);
    void setData(uint8_t num1, uint8_t num2);
    void setData(uint8_t num1, uint8_t num2, uint8_t num3, uint8_t num4);

    /**
     * @brief Float to uint8_t conversion, default big endian
     *
     * @param num1
     */
    void setData_impFloatCast(float num1);

    /**
     * @brief Float to uint8_t conversion, with endian option
     *
     * @param num1
     * @param endian , true for big endian, false for little endian
     */
    void setData_impFloatCast(float num1, bool endian);

    /**
     * @brief uint16_t into package data by splitting big endian to two bytes
     *
     * @param num1
     */
    void setData_impSplit(uint16_t num1);

    // IO
    bool importPacket(uint8_t* packet, uint16_t packetSize);
    bool exportPacket(uint8_t* packetBuffer, uint16_t packetBufferSize);

    Packet
    swapReply();  // Swap the host and client address octets, return a new packet for the reply

    // Checksum

    uint16_t calculateChecksum();         // Calculate the checksum of the packet
    void setChecksum(uint16_t checksum);  // Set the checksum of the packet

    bool validateChecksum();  // Validate the checksum of the packet matches the calculated checksum
};

class sysAdminPacket : public Packet {
   protected:
    uint16_t _adminMetaData;   // extra data for sysadmin packets providing additional information
    uint8_t _originHostOctet;  // The host octet of the origin of the packet

   public:
    // Constructor
    sysAdminPacket(uint8_t hostAddressOctet, uint8_t clientAddressOctet, uint8_t originHostOctet,
                   uint16_t actionCode, uint8_t* data, uint16_t dataBufferSize,
                   uint16_t adminMetaData);
    sysAdminPacket(uint8_t hostAddressOctet, uint8_t clientAddressOctet, uint8_t originHostOctet,
                   uint16_t actionCode, uint16_t adminMetaData);
    sysAdminPacket(uint8_t hostAddressOctet, uint8_t clientAddressOctet);
    sysAdminPacket();

    // Destructor
    ~sysAdminPacket();

    // Getters
    uint16_t getAdminMetaData();
    uint8_t getOriginHostOctet();

    // Setters
    void setAdminMetaData(uint16_t adminMetaData);
    void setOriginHostOctet(uint8_t originHostOctet);

    // IO
    bool importPacket(uint8_t* packet, uint16_t packetSize);
    bool exportPacket(uint8_t* packetBuffer, uint16_t packetBufferSize);

    sysAdminPacket
    swapReply();  // Swap the host and client address octets, return a new packet for the

    // Checksum
    uint16_t
    calculateChecksum();  // Calculate the checksum of the packet, including the adminMetaData
    // Validate the checksum function is inherited from the Packet class
};

}  // namespace ROIPackets
#endif  // PACKETS_H