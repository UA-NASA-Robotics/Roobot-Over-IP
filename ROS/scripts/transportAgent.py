#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from roi_ros.msg import SerializedPacket
from roi_ros.srv import QueueSerializedGeneralPacket, QueueSerializedSysAdminPacket
from roi_ros.action import SendSerializedSysAdminPacket, SendSerializedGeneralPacket

import socket, threading, time

GENERALPORT = 57344
SYSADMINPORT = 57664
ROIMAXPACKETSIZE = 60


def get_host_ip():
    """Get the local IP address of the machine

    Returns:
        str: The local IP address
    """
    try:
        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connect to a remote server to determine the local IP address
        s.connect(("8.8.8.8", 80))
        # Retrieve the IP address
        ip = s.getsockname()[0]
    except Exception as e:
        print(f"Error: {e}")
        ip = None
    finally:
        s.close()
    return ip


class TransportAgent(Node):
    def __init__(self):
        super().__init__("transport_agent")  # name node

        # Create publishers for each octet, 1-254, [0] is none.
        self.octetResponsePublishers = [
            self.create_publisher(SerializedPacket, "octet" + str(i + 1) + "_response", 10)
            for i in range(253)
        ]
        self.octetResponsePublishers.insert(0, None)

        self.sysAdminOctetResponsePublishers = [
            self.create_publisher(
                SerializedPacket, "sys_admin_octet" + str(i + 1) + "_response", 10
            )
            for i in range(253)
        ]
        self.sysAdminOctetResponsePublishers.insert(0, None)

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
        # - Format: [{'packet': [SerializedPacket;int], 'uid': int, 'status': int, 'octet': int, 'sentTimestamp': float}]
        self.generalPacketQueue = []
        self.sysAdminPacketQueue = []

        # parameters for the ROI system, not ros interconnect.
        self.declare_parameter("timeout", 0.05)  # timeout in seconds
        self.declare_parameter("max_retries", 1000)  # number of retries before abandoning packet
        # generally if a packet is abandoned, then data is lost. This is a last resort to keep the system from hanging.
        # Adjust the timeout to stop lost packets, or improve network connectivity.

        self.networkAddress = get_host_ip()  # get the local IP address of the machine

        # Create network sockets for general and sys admin packets
        self.generalNetworkSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.generalNetworkSocket.settimeout(self.get_parameter("timeout").value)
        self.generalNetworkSocket.bind((self.networkAddress, GENERALPORT))

        self.sysAdminNetworkSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sysAdminNetworkSocket.settimeout(self.get_parameter("timeout").value)
        self.sysAdminNetworkSocket.bind((self.networkAddress, SYSADMINPORT))

        # Create threads for listening and transmitting packets, follows the queueing system
        self.netGeneralListenerThread = threading.Thread(
            target=self.netListener, daemon=True, args=(self.generalNetworkSocket,)
        )
        self.netGeneralListenerThread.start()
        self.netSysAdminListenerThread = threading.Thread(
            target=self.netListener, daemon=True, args=(self.sysAdminNetworkSocket,)
        )
        self.netSysAdminListenerThread.start()

        self.netTransmitterThread = threading.Thread(target=self.netTransmitter, daemon=True)
        self.netTransmitterThread.start()

        # log initialization
        self.get_logger().info("Transport Agent Initialized")

    def queueGeneralPacketCallback(self, request, response):
        """Accepts a general packet and queues it for sending

        Args:
            request (roi_ros.srv.QueueSerializedGeneralPacket_Request): The request object
            response (roi_ros.srv.QueueSerializedGeneralPacket_Response): The response object
        """
        self.get_logger().info("Received general packet")
        response.success = True

        self.generalPacketQueue.append(
            {
                "packet": request.packet.data,
                "uid": self.generateGeneralPacketUID(request.packet.data),
                "status": 0,
                "octet": request.packet.client_octet,
            }
        )

    def queueSysAdminPacketCallback(self, request, response):
        """Accepts a sys admin packet and queues it for sending

        Args:
            request (roi_ros.srv.QueueSerializedSysAdminPacket_Request): The request object
            response (roi_ros.srv.QueueSerializedSysAdminPacket_Response): The response object
        """
        self.get_logger().info("Received sys admin packet")
        response.success = True

        self.sysAdminPacketQueue.append(
            {
                "packet": request.packet.data,
                "uid": self.generateSysAdminPacketUID(request.packet.data),
                "status": 0,
                "octet": request.packet.client_octet,
            }
        )

    def generateGeneralPacketUID(self, packetList):
        """Generates a UID for a general packet

        Args:
            packetList (list): The packet data

        Returns:
            int: The generated UID
        """
        return (
            (packetList[0] + packetList[1] * 7 + packetList[2] * 11 + packetList[3] * 13)
            * 17
            // 1024
        )

    def generateSysAdminPacketUID(self, packetList):
        """Generates a UID for a sys admin packet

        Args:
            packetList (list): The packet data

        Returns:
            int: The generated UID
        """
        return (packetList[2] * 7 + packetList[3] * 11 + packetList[4] * 13) * 19 // 1024

    def sendToOctet(self, octet, socket, packet, port):
        """Sends a packet to a specific octet based on the networkAddress

        Args:
            octet (int): The octet to send the packet to
            socket (socket): The socket to use
            packet (list): The packet data
        """
        socket.sendto(
            packet,
            (self.networkAddress.replace(self.networkAddress.split(".")[-1], str(octet)), port),
        )

    def netListener(self, socket):
        """Listens for incoming packets on the network"""
        while rclpy.ok():
            data, addr = socket.recvfrom(ROIMAXPACKETSIZE)
            if addr[0] == self.networkAddress:
                continue

            data = [int.from_bytes(data[i]) for i in range(len(data))]  # Convert bytes to int

            uid = (
                self.generateGeneralPacketUID(data)
                if socket == self.generalNetworkSocket
                else self.generateSysAdminPacketUID(data)
            )

            if socket == self.generalNetworkSocket:
                i = 0
                while i < len(self.generalPacketQueue):
                    if (
                        self.generalPacketQueue[i]["uid"] == uid
                        and addr[0].split(".")[3].strip() == self.generalPacketQueue[i]["octet"]
                    ):
                        self.generalPacketQueue.pop(i)
                        i = -1
                        break
                    i += 1
                if i != -1:
                    self.get_logger().info(
                        f"Received unwarranted general response from {addr[0]}. Timeout: {self.get_parameter('timeout').value} may be too short."
                    )

                try:
                    self.octetResponsePublishers[int(addr[0].split(".")[3].strip())].publish(
                        SerializedPacket(
                            data=data,
                            client_octet=int(addr[0].split(".")[3].strip()),
                            length=len(data),
                        )
                    )
                except Exception as e:
                    self.get_logger().error(f"Error: {e} at publishing general response")

            elif socket == self.sysAdminNetworkSocket:
                i = 0
                while i < len(self.sysAdminPacketQueue):
                    if (
                        self.sysAdminPacketQueue[i]["uid"] == uid
                        and addr[0].split(".")[3].strip() == self.sysAdminPacketQueue[i]["octet"]
                    ):
                        self.sysAdminPacketQueue.pop(i)
                        i = -1
                        break
                    i += 1
                if i != -1:
                    self.get_logger().info(
                        f"Received unwarranted sys admin response from {addr[0]}. Timeout: {self.get_parameter('timeout').value} may be too short."
                    )

                try:
                    self.sysAdminOctetResponsePublishers[
                        int(addr[0].split(".")[3].strip())
                    ].publish(
                        SerializedPacket(
                            data=data,
                            client_octet=int(addr[0].split(".")[3].strip()),
                            length=len(data),
                        )
                    )
                except Exception as e:
                    self.get_logger().error(f"Error: {e} at publishing sys admin response")

    def netTransmitter(
        self,
    ):
        """Transmits packets over the network"""
        while rclpy.ok():
            timeout = self.get_parameter("timeout").value
            retries = self.get_parameter("max_retries").value

            if len(self.generalPacketQueue) > 0:
                for packet in self.generalPacketQueue:
                    if packet["status"] == 0:
                        self.sendToOctet(
                            packet["octet"],
                            self.generalNetworkSocket,
                            packet["packet"],
                            GENERALPORT,
                        )
                        packet["status"] += 1
                        packet["sentTimestamp"] = time.time()

                    elif (
                        packet["status"] >= 1
                        and packet["status"] < retries
                        and time.time() - packet["sentTimestamp"] >= timeout
                    ):
                        self.sendToOctet(
                            packet["octet"],
                            self.generalNetworkSocket,
                            packet["packet"],
                            GENERALPORT,
                        )
                        packet["status"] += 1
                        packet["sentTimestamp"] = time.time()
                    else:
                        self.get_logger().info(
                            f"General packet to octet {packet['octet']} hit max retries and was abandoned. Increase timeout or max retries. Network Virtualization may be stale."
                        )
                        self.generalPacketQueue.remove(packet)

            if len(self.sysAdminPacketQueue) > 0:
                for packet in self.sysAdminPacketQueue:
                    if packet["status"] == 0:
                        self.sendToOctet(
                            packet["octet"],
                            self.sysAdminNetworkSocket,
                            packet["packet"],
                            SYSADMINPORT,
                        )
                        packet["status"] += 1
                        packet["sentTimestamp"] = time.time()

                    elif (
                        packet["status"] >= 1
                        and packet["status"] < retries
                        and time.time() - packet["sentTimestamp"] >= timeout
                    ):
                        self.sendToOctet(
                            packet["octet"],
                            self.sysAdminNetworkSocket,
                            packet["packet"],
                            SYSADMINPORT,
                        )
                        packet["status"] += 1
                        packet["sentTimestamp"] = time.time()
                    else:
                        self.get_logger().info(
                            f"Sys admin packet to octet {packet['octet']} hit max retries and was abandoned. Increase timeout or max retries. Network Virtualization may be stale."
                        )
                        self.sysAdminPacketQueue.remove(packet)


def main(args=None):
    rclpy.init(args=args)

    transport_agent = TransportAgent()

    rclpy.spin(transport_agent)

    transport_agent.destroy_node()
    rclpy.shutdown()
