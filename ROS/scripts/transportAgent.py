#!/usr/bin/env python3
import re
import rclpy
from rclpy.node import Node

# from rclpy.action import ActionServer

from roi_ros.msg import SerializedPacket, ConnectionState
from roi_ros.srv import QueueSerializedGeneralPacket, QueueSerializedSysAdminPacket

# from roi_ros.action import SendSerializedSysAdminPacket, SendSerializedGeneralPacket

import socket, threading, time, multiprocessing

import reliabilityManager, netTransmitter, netReceiver

# import psutil, os; psutil.Process(os.getpid()).nice(psutil.REALTIME_PRIORITY_CLASS) #force high compute priority

GENERALPORT = 57344
SYSADMINPORT = 57664
ROI_MAX_PACKET_SIZE = 64


class TransportAgent(Node):
    def __init__(self):
        super().__init__("transport_agent")  # name node

        # Create publishers for each octet, 1-254, [0] is none.
        self.octetResponsePublishers = [
            None
        ] * 255  # by default all are None see dynamicCreateTopics

        self.sysAdminOctetResponsePublishers = [
            None
        ] * 255  # by default all are None see dynamicCreateTopics

        self.octetConnectionStatePublishers = [None] * 255

        # Create services for accepting packets
        self.queueGeneralPacketService = self.create_service(
            QueueSerializedGeneralPacket, "queue_general_packet", self.queueGeneralPacketCallback
        )
        self.queueSysAdminPacketService = self.create_service(
            QueueSerializedSysAdminPacket,
            "queue_sys_admin_packet",
            self.queueSysAdminPacketCallback,
        )

        # Create queues for packets
        self.outgoingGeneralPacketQueue = multiprocessing.JoinableQueue()
        self.outgoingSysAdminPacketQueue = multiprocessing.JoinableQueue()

        self.incomingGeneralPacketQueue = multiprocessing.JoinableQueue()
        self.incomingSysAdminPacketQueue = multiprocessing.JoinableQueue()

        self.rxGeneralPacketQueue = multiprocessing.JoinableQueue()
        self.rxSysAdminPacketQueue = multiprocessing.JoinableQueue()

        self.txGeneralPacketQueue = multiprocessing.JoinableQueue()
        self.txSysAdminPacketQueue = multiprocessing.JoinableQueue()

        self.connectionQueue = multiprocessing.JoinableQueue()

        # Create processor threads
        self.transmitPool = [None] * 5
        self.sysAdminTransmitPool = [None] * 5

        self.generalReliabilityManager = None
        self.sysAdminReliabilityManager = None
        self.generalReceiver = None
        self.sysAdminReceiver = None

        # parameters for the ROI system, not ros interconnect.
        self.declare_parameter("timeout", 0.05)  # timeout in seconds
        self.declare_parameter("max_retries", 10)  # number of retries before abandoning packet
        self.declare_parameter("lost_to_disconnect", 1)
        self.declare_parameter(
            "network_address", "Null"
        )  # number of lost packets before reporting disconnect
        # generally if a packet is abandoned, then data is lost. This is a last resort to keep the system from hanging.
        # Adjust the timeout to stop lost packets, or improve network connectivity.
        self.declare_parameter(
            "transmit_threads", 5
        )  # number of multi-processing to spin up. (For general packets, only 1 sysadmin is spun)

        self.parameterWatcherThread = threading.Thread(target=self.parameterWatcher, daemon=True)
        self.parameterWatcherThread.start()

        self.connectionPublishThread = threading.Thread(
            target=self.connectionPublisher, daemon=True
        )
        self.connectionPublishThread.start()

        self.generalResponsePublishThread = threading.Thread(
            target=self.generalResponsePublisher, daemon=True
        )
        self.generalResponsePublishThread.start()
        self.sysAdminResponsePublishThread = threading.Thread(
            target=self.sysAdminResponsePublisher, daemon=True
        )
        self.sysAdminResponsePublishThread.start()

        self.queueLenWarnThread = threading.Thread(target=self.queueLenWarn, daemon=True)
        self.queueLenWarnThread.start()

        # log initialization
        self.get_logger().info("Transport Agent Initialized")

    def queueGeneralPacketCallback(self, request, response):
        """Accepts a general packet and queues it for sending

        Args:
            request (roi_ros.srv.QueueSerializedGeneralPacket_Request): The request object
            response (roi_ros.srv.QueueSerializedGeneralPacket_Response): The response object
        """
        # self.get_logger().info("Received general packet")
        response.success = True

        self.txGeneralPacketQueue.put(
            {
                "packet": request.packet.data,
                "octet": request.packet.client_octet,
            }
        )

        return response

    def queueSysAdminPacketCallback(self, request, response):
        """Accepts a sys admin packet and queues it for sending

        Args:
            request (roi_ros.srv.QueueSerializedSysAdminPacket_Request): The request object
            response (roi_ros.srv.QueueSerializedSysAdminPacket_Response): The response object
        """
        # self.get_logger().info("Received general packet")
        response.success = True

        self.txSysAdminPacketQueue.put(
            {
                "packet": request.packet.data,
                "octet": request.packet.client_octet,
            }
        )

        return response

    def connectionPublisher(self):
        """Thread to publish connection state of octets as they are queued to update"""

        while rclpy.ok():
            # Check for connection state changes
            connectionState = self.connectionQueue.get(block=True)

            # pull out the information from the connection state
            octet = connectionState["octet"]
            lost_packets_since_connect = connectionState["lost_packets_since_connect"]
            lost_packets_accumulated = connectionState["lost_packets_accumulated"]
            octet_connected = connectionState["octet_connected"]

            # Check if the publisher for the octet exists, if not create it
            if self.octetConnectionStatePublishers[octet] is None:
                self.dynamicCreateTopics(octet)

            # Publish the connection state
            connectionStateMsg = ConnectionState()
            connectionStateMsg.module_connected = octet_connected
            connectionStateMsg.lost_packets_since_connect = lost_packets_since_connect
            connectionStateMsg.lost_packets_accumulated = lost_packets_accumulated

            self.octetConnectionStatePublishers[octet].publish(connectionStateMsg)

    def generalResponsePublisher(self):
        """Thread to publish general response packets as they are queued to update"""

        while rclpy.ok():
            # Check for response packets
            packetDict = self.rxGeneralPacketQueue.get(block=True)

            # pull out the information from the packet
            octet = packetDict["octet"]
            packet = packetDict["packet"]

            # Check if the publisher for the octet exists, if not create it
            if self.octetResponsePublishers[octet] is None:
                self.dynamicCreateTopics(octet)

            # Publish the response packet
            serializedPacket = SerializedPacket()
            serializedPacket.data = packet
            serializedPacket.client_octet = octet
            serializedPacket.length = ROI_MAX_PACKET_SIZE

            self.octetResponsePublishers[octet].publish(serializedPacket)

    def sysAdminResponsePublisher(self):
        """Thread to publish sys admin response packets as they are queued to update"""

        while rclpy.ok():
            # Check for response packets
            packetDict = self.rxSysAdminPacketQueue.get(block=True)

            # pull out the information from the packet
            octet = packetDict["octet"]
            packet = packetDict["packet"]

            # Check if the publisher for the octet exists, if not create it
            if self.sysAdminOctetResponsePublishers[octet] is None:
                self.dynamicCreateTopics(octet)

            # Publish the response packet
            serializedPacket = SerializedPacket()
            serializedPacket.data = packet
            serializedPacket.client_octet = octet
            serializedPacket.length = ROI_MAX_PACKET_SIZE

            self.sysAdminOctetResponsePublishers[octet].publish(serializedPacket)

    def parameterWatcher(self):
        """Watches for the network address parameter to change
        Updates network sockets.
        """

        def reInitProcessors(self):
            """Reinitalizes all tx, rx and reliability manager threads"""

            for tx in self.transmitPool:
                if tx is not None:
                    tx.terminate()

            for tx in self.sysAdminTransmitPool:
                if tx is not None:
                    tx.terminate()

            if self.generalReliabilityManager is not None:
                self.generalReliabilityManager.terminate()

            if self.sysAdminReliabilityManager is not None:
                self.sysAdminReliabilityManager.terminate()

            if self.generalReceiver is not None:
                self.generalReceiver.terminate()

            if self.sysAdminReceiver is not None:
                self.sysAdminReceiver.terminate()

            # Get parameters
            network_address = self.get_parameter("network_address").value
            timeout = self.get_parameter("timeout").value
            max_retries = self.get_parameter("max_retries").value
            lost_to_disconnect = self.get_parameter("lost_to_disconnect").value
            transmit_threads = self.get_parameter("transmit_threads").value

            # Create new tx pools
            self.transmitPool = [
                multiprocessing.Process(
                    target=netTransmitter.netTransmitter,
                    args=(self.txGeneralPacketQueue, network_address, timeout, GENERALPORT),
                    daemon=True,
                )
                for i in range(transmit_threads)
            ]
            self.sysAdminTransmitPool = [
                multiprocessing.Process(
                    target=netTransmitter.netTransmitter,
                    args=(self.txSysAdminPacketQueue, network_address, timeout, SYSADMINPORT),
                    daemon=True,
                )
                for i in range(transmit_threads)
            ]
            for i in range(transmit_threads):
                self.transmitPool[i].start()
                self.sysAdminTransmitPool[i].start()

            # Create new reliability managers
            self.generalReliabilityManager = multiprocessing.Process(
                target=reliabilityManager.reliabilityManager,
                args=(
                    self.incomingGeneralPacketQueue,
                    self.outgoingGeneralPacketQueue,
                    self.txGeneralPacketQueue,
                    self.rxGeneralPacketQueue,
                    self.connectionQueue,
                    reliabilityManager.generateGeneralPacketUID,
                    timeout,
                    max_retries,
                    lost_to_disconnect,
                ),
                daemon=True,
            )
            self.sysAdminReliabilityManager = multiprocessing.Process(
                target=reliabilityManager.reliabilityManager,
                args=(
                    self.incomingSysAdminPacketQueue,
                    self.outgoingSysAdminPacketQueue,
                    self.txSysAdminPacketQueue,
                    self.rxSysAdminPacketQueue,
                    self.connectionQueue,
                    reliabilityManager.generateSysAdminPacketUID,
                    timeout,
                    max_retries,
                    lost_to_disconnect,
                ),
                daemon=True,
            )
            self.generalReliabilityManager.start()
            self.sysAdminReliabilityManager.start()

            # Create new receivers
            self.generalReceiver = multiprocessing.Process(
                target=netReceiver.netReceiver,
                args=(self.rxGeneralPacketQueue, timeout, GENERALPORT, ROI_MAX_PACKET_SIZE),
                daemon=True,
            )
            self.sysAdminReceiver = multiprocessing.Process(
                target=netReceiver.netReceiver,
                args=(self.rxSysAdminPacketQueue, timeout, SYSADMINPORT, ROI_MAX_PACKET_SIZE),
                daemon=True,
            )
            self.generalReceiver.start()
            self.sysAdminReceiver.start()
            self.get_logger().info("Reinitialized transmitters and receivers")

        lastKnownParameters = {
            "network_address": "abcd",
            "timeout": 0,
            "max_retries": 0,
            "lost_to_disconnect": 0,
            "transmit_threads": 0,
        }

        while rclpy.ok():
            # Check for parameter changes
            redo = False
            for param in lastKnownParameters.keys():
                if self.get_parameter(param).value != lastKnownParameters[param]:
                    lastKnownParameters[param] = self.get_parameter(param).value
                    self.get_logger().info(
                        f"Parameter {param} changed to {lastKnownParameters[param]}. Reinitializing udp processors..."
                    )
                    redo = True
            if redo:
                reInitProcessors(self)

            time.sleep(2)  # Sleep for a bit to avoid busy waiting

    def dynamicCreateTopics(self, octet):
        """Dynamically creates topics for the given octet. (Checks none exist)

        Args:
            octet (int): The octet to create topics for
        """

        if self.octetResponsePublishers[octet] is None:
            self.octetResponsePublishers[octet] = self.create_publisher(
                SerializedPacket, "octet" + str(octet) + "_response", 10
            )
            self.sysAdminOctetResponsePublishers[octet] = self.create_publisher(
                SerializedPacket, "sys_admin_octet" + str(octet) + "_response", 10
            )
            self.octetConnectionStatePublishers[octet] = self.create_publisher(
                ConnectionState, "octet" + str(octet) + "_connection_state", 10
            )
            self.get_logger().info(f"Created topics for octet {octet}")

    def queueLenWarn(self):
        """Checks the queue lengths and warns if they are too long"""

        while rclpy.ok():
            if self.txGeneralPacketQueue.qsize() > 10:
                self.get_logger().warn(
                    f"General Queue Long: {self.txGeneralPacketQueue.qsize()} awaiting."
                )

            if self.txSysAdminPacketQueue.qsize() > 10:
                self.get_logger().warn(
                    f"SysAdmin Queue Long: {self.txSysAdminPacketQueue.qsize()} awaiting."
                )
            if self.incomingGeneralPacketQueue.qsize() > 10:
                self.get_logger().warn(
                    f"Incoming General Queue Long: {self.incomingGeneralPacketQueue.qsize()} awaiting."
                )
            if self.incomingSysAdminPacketQueue.qsize() > 10:
                self.get_logger().warn(
                    f"Incoming SysAdmin Queue Long: {self.incomingSysAdminPacketQueue.qsize()} awaiting."
                )
            if self.outgoingGeneralPacketQueue.qsize() > 10:
                self.get_logger().warn(
                    f"Outgoing General Queue Long: {self.outgoingGeneralPacketQueue.qsize()} awaiting."
                )
            if self.outgoingSysAdminPacketQueue.qsize() > 10:
                self.get_logger().warn(
                    f"Outgoing SysAdmin Queue Long: {self.outgoingSysAdminPacketQueue.qsize()} awaiting."
                )
            if self.rxGeneralPacketQueue.qsize() > 10:
                self.get_logger().warn(
                    f"Rx General Queue Long: {self.rxGeneralPacketQueue.qsize()} awaiting."
                )
            if self.rxSysAdminPacketQueue.qsize() > 10:
                self.get_logger().warn(
                    f"Rx SysAdmin Queue Long: {self.rxSysAdminPacketQueue.qsize()} awaiting."
                )

            time.sleep(1)
            # Sleep for a bit to avoid busy waiting


def main(args=None):
    rclpy.init(args=args)

    transport_agent = TransportAgent()

    rclpy.spin(transport_agent)

    transport_agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
