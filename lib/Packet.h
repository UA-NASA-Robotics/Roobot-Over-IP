#ifndef PACKETS_H
#define PACKETS_H

// #include <vector> //Not implemented in Arduino, plus we need to be more memory efficient
#include <stdint.h>
namespace ROIConstants {
// Constants for the ROI module

const uint16_t ROIGENERALPORT = 57344;   // The port that the ROI module listens on for general
                                         // packets. This is the first port in the range 2^16 - 2^13
const uint16_t ROIINTERUPTPORT = 57600;  // The port that the ROI module listens on for interrupt
                                         // packets. This is the second port in the range
const uint16_t ROISYSADMINPORT = 57664;  // The port that the ROI module listens on for sysAdmin
                                         // packets. This is the third port in the range

const uint8_t ROIMAXPACKETPAYLOAD = 19;  // The maximum size of a packet data payload in bytes
const uint8_t ROIMAXPACKETSIZE = 24;     // The maximum size of a packet in bytes
}  // namespace ROIConstants

namespace ROIPackets {
class Packet {
   protected:
    uint32_t networkAddress;     // 4 bytes IP address
    uint8_t hostAddressOctet;    // 1 byte host address
    uint8_t clientAddressOctet;  // 1 byte client address

    uint16_t subDeviceID;                             // 2 bytes subdevice ID
    uint16_t actionCode;                              // 2 bytes action code
    uint8_t data[ROIConstants::ROIMAXPACKETPAYLOAD];  // 0-19 bytes of data

   public:
    // Constructor
    Packet(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
           uint16_t subDeviceID, uint16_t actionCode, uint8_t *data);
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
    void getData(uint8_t *dataBuffer);

    // Setters
    void setNetworkAddress(uint32_t networkAddress);
    void setHostAddressOctet(uint8_t hostAddressOctet);
    void setClientAddressOctet(uint8_t clientAddressOctet);

    void setSubDeviceID(uint16_t subDeviceID);
    void setActionCode(uint16_t actionCode);
    void setData(uint8_t *data);

    // IO
    bool importPacket(uint8_t *packet);
    bool exportPacket(uint8_t *packetBuffer);
};

class sysAdminPacket : public Packet {
   protected:
    uint16_t adminMetaData;  // extra data for sysadmin packets providing additional information

   public:
    // Constructor
    sysAdminPacket(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
                   uint16_t actionCode, uint8_t *data, uint16_t adminMetaData);
    sysAdminPacket();

    // Destructor
    ~sysAdminPacket();

    // Getters
    uint16_t getAdminMetaData();

    // Setters
    void setAdminMetaData(uint16_t adminMetaData);

    // IO
    bool importPacket(uint8_t *packet);
    bool exportPacket(uint8_t *packetBuffer);
};

}  // namespace ROIPackets
#endif  // PACKETS_H