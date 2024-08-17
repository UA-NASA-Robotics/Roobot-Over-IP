#include "transportAgent.h"

/*---- Private Functions ----*/
void TransportAgent::transportAgentWorker() {
    // The worker function for the transport agent thread
    while (true) {
        // Send all general packets
    }
}

uint32_t TransportAgent::generateGeneralPacketUID(ROIPackets::Packet packet) {
    uint32_t uid = 0;
    uid +=
        packet
            .getHostAddressOctet();  // make these interchangable as they will be swapped on receive
    uid += packet.getClientAddressOctet();

    uid *= 17;  // Prime number to spread out the values

    uid += packet.getSubDeviceID();  // this stays the same, so doesn't need to be interchangable
    uid *= 11;                       // Prime number
    uid += packet.getActionCode();
    uid *= 13;  // Prime number

    return uid;
}

uint32_t TransportAgent::generateSysAdminPacketUID(ROIPackets::sysAdminPacket packet) {
    uint32_t uid = 0;
    uid +=
        packet
            .getHostAddressOctet();  // make these interchangable as they will be swapped on receive
    uid += packet.getClientAddressOctet();

    uid *= 17;  // Prime number to spread out the values

    uid += packet.getOriginOctet();  // this stays the same, so doesn't need to be interchangable
    uid *= 11;                       // Prime number
    uid += packet.getActionCode();
    uid *= 13;  // Prime number

    return uid;
}

/*---- Public Functions ----*/

TransportAgent::TransportAgent() {}

TransportAgent::~TransportAgent() {
    // Clear the module vector
    modules.clear();
}

void TransportAgent::init() {
    // Initialize the transport agent thread
    transportAgentThread = std::thread(&TransportAgent::transportAgentWorker, this);
    transportAgentThread.detach();
}

void TransportAgent::pushModule(BaseModule* module) {
    // Push a module to the transport agent
    modules.push_back(module);
}

bool TransportAgent::removeModule(BaseModule* module) {
    // Remove a module from the transport agent
    for (int i = 0; i < modules.size(); i++) {
        if (modules[i] == module) {
            modules.erase(modules.begin() + i);
            return true;
        }
    }
    return false;
}

void TransportAgent::queueGeneralPacket(ROIPackets::Packet packet) {
    // Queue a general packet to be sent to the modules
    generalPacketQueue.push_back(packet);
}

void TransportAgent::queueSysAdminPacket(ROIPackets::sysAdminPacket packet) {
    // Queue a sysAdmin packet to be sent to the modules
    sysAdminPacketQueue.push_back(packet);
}
