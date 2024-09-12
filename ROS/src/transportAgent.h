#ifndef TRANSPORTAGENT_H
#define TRANSPORTAGENT_H

#include <iostream>
#include <thread>
#include <vector>

#include "../../lib/ModuleCodec.h"
#include "../../lib/Packet.h"
#include "moduleVirtualizations/base.h"
#include "rclcpp/rclcpp.hpp"
#include "roi_ros/msg/serialized_packet.hpp"
#include "socketwrapper-2/include/socketwrapper/endpoint.hpp"
#include "socketwrapper-2/include/socketwrapper/socketwrapper.hpp"
#include "socketwrapper-2/include/socketwrapper/span.hpp"
#include "socketwrapper-2/include/socketwrapper/udp.hpp"
#include "socketwrapper-2/include/socketwrapper/utility.hpp"

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

    std::vector<array<uint8_t, ROIConstants::ROIMAXPACKETSIZE>>
        generalPacketQueue;  // Queue of packets to be sent to the modules

    std::vector<uint32_t> generalPacketUIDQueue;  // Queue of UIDs of packets to

    std::vector<bool>
        generalPacketQueueStatus;  // Queue of packet statuses to be sent to the modules

    std::vector<array<uint8_t, ROIConstants::ROIMAXPACKETSIZE>>
        sysAdminPacketQueue;  // Queue of sysAdmin packets to be sent to the modules

    std::vector<uint32_t>
        sysAdminPacketUIDQueue;  // Queue of UIDs of sysAdmin packets to be sent to the modules

    std::vector<bool>
        sysAdminPacketQueueStatus;  // Queue of sysAdmin packet statuses to be sent to the modules

    /**
     * @brief  Generates a unique ID for a general packet
     *
     * @param packet , the packet to generate a unique ID for
     * @return uint32_t, the UID
     */
    uint32_t generateGeneralPacketUID(ROIPackets::Packet packet);

    /**
     * @brief Generates a unique ID for a sysAdmin packet
     *
     * @param packet , the packet to generate a unique ID for
     * @return uint32_t, the UID
     */
    uint32_t generateSysAdminPacketUID(ROIPackets::sysAdminPacket packet);

    /**
     * @brief Generates a unique ID for a packet from a serialized packet
     *
     * @param packet , the serialized packet in a uint8_t array
     * @param packetSize , the size of the packet
     * @return uint32_t, the UID
     */
    uint32_t generateUIDFromSerializedPacket(uint8_t* packet, uint16_t packetSize);

    /**
     * @brief The worker function for the transport agent, to be run in a separate thread
     *
     */
    void transportAgentWorker();

    std::thread transportAgentThread;  // Thread for the transport agent worker

    endpoint_v4 generalSCBEndpoint;   // Endpoint for the SBC
    endpoint_v4 sysAdminSCBEndpoint;  // Endpoint for the SBC

    udp_socket_v4 generalSocket;   // Socket for general packets
    udp_socket_v4 sysAdminSocket;  // Socket for sysAdmin packets

    uint8_t generalBuffer[ROIConstants::MAXPACKETSIZE];  // Buffer for reading packets. Shared
                                                         // between general and sysAdmin packets,
                                                         // but only in worker thread

    endpoint_v4* generalModuleEndPoints[255];   // Array of endpoints for the modules
    endpoint_v4* sysAdminModuleEndPoints[255];  // Array of endpoints for the modules

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