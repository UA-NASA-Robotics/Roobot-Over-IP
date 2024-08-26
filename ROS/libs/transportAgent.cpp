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

    uid +=
        packet.getOriginHostOctet();  // this stays the same, so doesn't need to be interchangable
    uid *= 11;                        // Prime number
    uid += packet.getActionCode();
    uid *= 13;  // Prime number

    return uid;
}

/*---- Public Functions ----*/

TransportAgent::TransportAgent(uint8_t* networkAddress) {
    // Constructor
    for (int i = 0; i < 4; i++) {
        this->networkAddress[i] = networkAddress[i];
    }
}

TransportAgent::~TransportAgent() {
    // Clear the module vector
    for (int i = 0; i < 255; i++) {
        if (modulesArray[i] != nullptr) {
            delete modulesArray[i];
        }
        if (moduleAliasArray[i] != "") {
            moduleAliasArray[i] = "";
        }
    }
}

void TransportAgent::init() {
    // Initialize the transport agent thread
    transportAgentThread = std::thread(&TransportAgent::transportAgentWorker, this);
    transportAgentThread.detach();
}

uint8_t TransportAgent::getHostAddressOctet() {
    // Get the host address octet of the SBC
    return networkAddress[3];
}

uint8_t TransportAgent::getAliasLookup(std::string alias) {
    // Get the host address octet of a module with a given alias
    for (int i = 0; i < 255; i++) {
        if (moduleAliasArray[i] == alias) {
            return i;
        }
    }
    return 0;
}

void TransportAgent::pushModule(BaseModule* module, std::string alias) {
    // Push a module to the transport agent
    modulesArray[module->getOctet()] = module;
    moduleAliasArray[module->getOctet()] = alias;
    uint8_t octet[4] = {networkAddress[0], networkAddress[1], networkAddress[2],
                        module->getOctet()};
    moduleEndPoints[module->getOctet()] = new endpoint(octet, ROIConstants::GENERALPORT);
}

bool TransportAgent::removeModule(uint8_t octet) {
    // Remove a module from the transport agent
    modulesArray[octet] = nullptr;
    moduleAliasArray[octet] = "";
    delete moduleEndPoints[octet];
}

void TransportAgent::queueGeneralPacket(ROIPackets::Packet packet) {
    // Queue a general packet to be sent to the modules
    generalPacketQueue.push_back(packet);
    generalPacketUIDQueue.push_back(generateGeneralPacketUID(packet));
    generalPacketQueueStatus.push_back(TransportAgentConstants::QUEUENOTSENT);
}

void TransportAgent::queueSysAdminPacket(ROIPackets::sysAdminPacket packet) {
    // Queue a sysAdmin packet to be sent to the modules
    sysAdminPacketQueue.push_back(packet);
    sysAdminPacketUIDQueue.push_back(generateSysAdminPacketUID(packet));
    sysAdminPacketQueueStatus.push_back(TransportAgentConstants::QUEUENOTSENT);
}
