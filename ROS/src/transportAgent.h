#ifndef TRANSPORTAGENT_H
#define TRANSPORTAGENT_H

#include <thread>
#include <vector>

#include "../../lib/ModuleCodec.h"
#include "../../lib/Packet.h"
#include "../libs/libSocket/headers/unixdgram.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roi_ros/msg/serialized_packet.hpp"

/*

This is the transport agent class. It is responsible for sending and receiving packets to and from
modules.

*/
namespace TransportAgentConstants {
constexpr uint8_t QUEUENOTSENT = 0;  // Packet has not been sent
constexpr uint8_t QUEUESENT = 1;     // Packet has been sent
}  // namespace TransportAgentConstants

class TransportAgent : public rclcpp::Node {
   private:
    uint8_t networkAddress[4];  // The network address of the SBC

    std::vector<std::array<uint8_t, ROIConstants::ROIMAXPACKETSIZE>>
        generalPacketQueue;  // Queue of packets to be sent to the modules

    std::vector<uint32_t> generalPacketUIDQueue;  // Queue of UIDs of packets to

    std::vector<bool>
        generalPacketQueueStatus;  // Queue of packet statuses to be sent to the modules

    std::vector<std::array<uint8_t, ROIConstants::ROIMAXPACKETSIZE>>
        sysAdminPacketQueue;  // Queue of sysAdmin packets to be sent to the modules

    std::vector<uint32_t>
        sysAdminPacketUIDQueue;  // Queue of UIDs of sysAdmin packets to be sent to the modules

    std::vector<bool>
        sysAdminPacketQueueStatus;  // Queue of sysAdmin packet statuses to be sent to the modules

    /**
     * @brief  Generates a unique ID for a general packet
     *
     * @param packet , the packet to generate a unique ID for (should be a uint8_t array)
     * @param packetSize , the size of the packet array <= ROIConstants::ROIMAXPACKETSIZE
     * @param clientAddressOctet , the client address octet of the packet
     * @return uint32_t, the UID
     */
    uint32_t generateGeneralPacketUID(uint8_t* packet, uint16_t packetSize,
                                      uint8_t clientAddressOctet);

    /**
     * @brief Generates a unique ID for a sysAdmin packet
     *
     * @param packet , the packet to generate a unique ID for (should be a uint8_t array)
     * @param packetSize , the size of the packet array <= ROIConstants::ROIMAXPACKETSIZE
     * @param clientAddressOctet , the client address octet of the packet
     * @return uint32_t, the UID
     */
    uint32_t generateSysAdminPacketUID(ROIPackets::sysAdminPacket packet);

    /**
     * @brief The worker function for the transport agent, to be run in a separate thread
     *
     */
    void transportAgentWorker();

    std::thread transportAgentThread;  // Thread for the transport agent worker

    uint8_t generalBuffer[ROIConstants::ROIMAXPACKETSIZE];  // Buffer for reading packets. Shared
                                                            // between general and sysAdmin packets,
                                                            // but only in worker thread

    rclcpp::Publisher<roi_ros::msg::SerializedPacket>::SharedPtr
        serializedPacketPublisherArray[255];
    // the vector of publishers for the serialized packets. Ie all the modules

   public:
    /**
     * @brief Construct a new Transport Agent object
     *
     * @param networkAddress , a uint8_t[4] array representing the network address of the SBC
     */
    TransportAgent(uint8_t* networkAddress);

    /**
     * @brief Destroy the Transport Agent object
     *
     */
    ~TransportAgent();

    /**
     * @brief Initializes the transport agent and all endpoints, starts the worker thread
     *
     */
    void init();

    /**
     * @brief Get the Host Address Octet of the SBC
     *
     * @return uint8_t, the host address octet
     */
    uint8_t getHostAddressOctet();

    /**
     * @brief Adds a general packet to the send queue
     *
     * @param packet , the general packet to add
     */
    void queueGeneralPacket(ROIPackets::Packet packet);

    /**
     * @brief Queues a sysAdmin packet to be sent
     *
     * @param packet , the sysAdmin packet to queue
     */
    void queueSysAdminPacket(ROIPackets::sysAdminPacket packet);

    /**
     * @brief Pushes a pre-serialized general packet to the queue
     *
     * @param packet , the packet to queue, as a uint8_t array
     * @param packetSize , the size of the packet array <= ROIConstants::ROIMAXPACKETSIZE
     */
    void queueSerializedGeneralPacket(uint8_t* packet, uint16_t packetSize);

    /**
     * @brief Pushes a pre-serialized sysAdmin packet to the queue
     *
     * @param packet , the packet to queue, as a uint8_t array
     * @param packetSize , the size of the packet array <= ROIConstants::ROIMAXPACKETSIZE
     */
    void queueSerializedSysAdminPacket(uint8_t* packet, uint16_t packetSize);
};

#endif