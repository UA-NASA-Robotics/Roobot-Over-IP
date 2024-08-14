#include "transportAgent.h"

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

void TransportAgent::transportAgentWorker() {
    // The worker function for the transport agent thread
    while (true) {
        // Send all general packets
    }
}