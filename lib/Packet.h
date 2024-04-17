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

namespace AdminConstants {
// Code and information to be used when building a SysAdminPacket
// Note there is no general packet constants. These should be with individual module sub-classes
// which can assign different meaning to code and data. The sysAdmin network is standardized for all
// modules.

// sysAdmin code bit n is a chain message flag, it should be '|' with any other code that must be
// passed along in the sysAdminPacket. Be careful when requesting all devices on a network to send a
// payload heavy response.

uint16_t NOCHAINMETA = 0;  // Metadata code for a sysAdminPacket that should not be circulated.
uint16_t CHAINMESSAGEMETA =
    32768;  // Metadata code for a sysAdminPacket that MUST be circulated around the module chain.

uint16_t PINGMETA = 0b0100000000000000;  // Metadata code for a admin Packet that should respond if
                                         // awake and ready, and a module identifier.

uint16_t STATUSREPORTMETA = 0b0010000000000000;  // Metadata code for a admin Packet that should
                                                 // elicit status information as a response.

/*--------- Module ID Codes ----------------*/
// We are skipping 0 and 1 as they may be used for fail or error codes
uint16_t MasterSBC = 2;    // The MasterSBC module returns a 2 as it's id in a ping
uint16_t GeneralGPIO = 3;  // A generalGPIO module returns a 3 as it's id in a ping

}  // namespace AdminConstants
class SysAdminPacket : public Packet {
   private:
    uint16_t adminMetaData;  // extra data for sysadmin packets providing additional information

   public:
    // Constructor
    SysAdminPacket(uint32_t networkAddress, uint8_t hostAddressOctet, uint8_t clientAddressOctet,
                   uint16_t subDeviceID, uint16_t actionCode, std::vector<uint8_t> data,
                   uint16_t adminMetaData);
    SysAdminPacket(uint32_t HostIPaddress, uint8_t clientAddressOctet, uint16_t subDeviceID,
                   uint16_t actionCode, std::vector<uint8_t> data, uint16_t adminMetaData);
    SysAdminPacket(uint16_t adminMetaData, uint16_t subDeviceID, uint16_t actionCode,
                   std::vector<uint8_t> data);
    SysAdminPacket();

    // Destructor
    ~SysAdminPacket();

    // Getters
    uint16_t getAdminMetaData();

    // Setters
    void setAdminMetaData(uint16_t adminMetaData);

    // IO
    bool importPacket(std::vector<uint8_t> packet);
    std::vector<uint8_t> exportPacket();

    // Print
    void printPacket();
};

}  // namespace ROIPackets
#endif  // PACKETS_H