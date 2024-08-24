#ifndef TRANSPORTAGENT_H
#define TRANSPORTAGENT_H

// #include <sys/socket.h>

#include <iostream>
#include <thread>
#include <vector>

#include "../../lib/ModuleCodec.h"
#include "../../lib/Packet.h"
#include "../../lib/UnityTypes.hpp"
#include "moduleVirtualizations/base.h"

/*

This is the transport agent class. It is responsible for sending and receiving packets to and from
modules. It is also responsible for maintaining the state of the modules.

*/
namespace TransportAgentConstants {}  // namespace TransportAgentConstants

class TransportAgent {
   private:
    std::vector<BaseModule*> modules;  // Vector of modules that the transport agent is responsible
                                       // for

    std::vector<ROIPackets::Packet>
        generalPacketQueue;  // Queue of packets to be sent to the modules

    std::vector<ROIPackets::sysAdminPacket>
        sysAdminPacketQueue;  // Queue of sysAdmin packets to be sent to the modules

    uint32_t generateGeneralPacketUID(
        ROIPackets::Packet packet);  // Generates a unique ID for a packet
    uint32_t generateSysAdminPacketUID(
        ROIPackets::sysAdminPacket packet);  // Generates a unique ID for a sysAdmin packet

    void transportAgentWorker();  // Worker function for the transport agent thread

    std::thread transportAgentThread;  // Thread for the transport agent worker

   public:
    TransportAgent();  // Constructor

    ~TransportAgent();  // Destructor

    void init();  // Initializes the transport agent thread

    // Getters/Setters
    uint8_t getHostAddressOctet();  // this SBC's host address octet

    // Public Use Functions

    void pushModule(BaseModule* module);  // Pushes a module to the transport agent

    bool removeModule(BaseModule* module);  // Removes a module from the transport agent

    void queueGeneralPacket(
        ROIPackets::Packet packet);  // Queues a packet to be sent to the modules

    void queueSysAdminPacket(
        ROIPackets::sysAdminPacket packet);  // Queues a sysAdmin packet to be sent to the modules
};

#endif