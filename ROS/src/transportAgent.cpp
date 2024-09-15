#include "transportAgent.h"

/*---- Private Functions ----*/
void TransportAgent::transportAgentWorker() {
    // The worker function for the transport agent thread

    while (true) {
        // Send all general packets

        for (int i = 0; i < generalPacketQueue.size(); i++) {
            if (generalPacketQueueStatus[i] == TransportAgentConstants::QUEUENOTSENT) {
                generalSocket.send(moduleEndPoints[generalPacketQueue[i].getClientAddressOctet()],
                                   generalPacketQueue[i].getPacketData());
                generalPacketQueueStatus[i] = TransportAgentConstants::QUEUESENT;
            }
        }
    }
}

uint32_t TransportAgent::generateGeneralPacketUID(uint8_t* packet, uint16_t packetSize,
                                                  uint8_t clientAddressOctet) {
    uint32_t uid = 0;
    uid += packet[0];
    uid += packet[1];
    uid *= 17;  // Prime number to spread out the values
    uid += packet[2];
    uid *= 11;  // Prime number
    uid += packet[3];
    uid *= 13;  // Prime number
    uid += clientAddressOctet;
    uid *= 19;  // Prime number
    return uid;
}

uint32_t TransportAgent::generateSysAdminPacketUID(uint8_t* packet, uint16_t packetSize,
                                                   uint8_t clientAddressOctet) {
    uint32_t uid = 0;
    uid += packet[2];
    uid += packet[3];
    uid *= 17;  // Prime number to spread out the values
    uid += packet[4];
    uid *= 11;  // Prime number
    uid += clientAddressOctet;
    uid *= 19;  // Prime number
    return uid;
}

/*---- Public Functions ----*/

TransportAgent::TransportAgent(uint8_t* networkAddress), rclcpp::Node("transportAgent") {
    // Constructor
    for (int i = 0; i < 4; i++) {
        this->networkAddress[i] = networkAddress[i];
    }

    for (int i = 0; i < 255; i++) {
        serializedPacketPublisherArray[i] = this->create_publisher<roi_ros::msg::SerializedPacket>(
            "octet" + std::to_string(i) + "response",
            10);  // initialize the publisher for the serialized packets with a history of 10
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

int main(int argc, char* argv[]) {
    // Main function for the node
    uint8_t networkAddress[4] = {192, 168, 1, 1};
    rclcpp::init(argc, argv);
    auto transportAgent = std::make_shared<TransportAgent>(networkAddress);
    transportAgent->init();
    rclcpp::spin(transportAgent);
    rclcpp::shutdown();
}