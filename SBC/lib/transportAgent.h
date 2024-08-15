#ifndef TRANSPORTAGENT_H
#define TRANSPORTAGENT_H

#include <sys/socket.h>

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
namespace TransportAgentConstants {
const int TRANSPORT_AGENT_SLEEP_TIME = 1000;  // Sleep time for the transport agent thread
}  // namespace TransportAgentConstants

// namespace TransportAgent {
class TransportAgent {
   private:
    std::vector<BaseModule*> modules;  // Vector of modules that the transport agent is responsible
                                       // for

    std::vector<ROIPackets::Packet>
        generalPacketQueue;  // Queue of packets to be sent to the modules

    std::vector<ROIPackets::sysAdminPacket>
        sysAdminPacketQueue;  // Queue of sysAdmin packets to be sent to the modules

    void transportAgentWorker();  // Worker function for the transport agent thread

    std::thread transportAgentThread;  // Thread for the transport agent worker

   public:
    TransportAgent();  // Constructor

    ~TransportAgent();  // Destructor

    void init();  // Initializes the transport agent thread

    void pushModule(BaseModule* module);  // Pushes a module to the transport agent

    bool removeModule(BaseModule* module);  // Removes a module from the transport agent

    void queueGeneralPacket(
        ROIPackets::Packet packet);  // Queues a packet to be sent to the modules

    void queueSysAdminPacket(
        ROIPackets::sysAdminPacket packet);  // Queues a sysAdmin packet to be sent to the modules
};
//}  // namespace TransportAgenta

#endif