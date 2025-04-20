#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# from rclpy.action import ActionServer

from roi_ros.msg import SerializedPacket, ConnectionState
from roi_ros.srv import QueueSerializedGeneralPacket, QueueSerializedSysAdminPacket

# from roi_ros.action import SendSerializedSysAdminPacket, SendSerializedGeneralPacket

import socket, threading, time

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
        # - Format: [{'packet': [SerializedPacket;int], 'uid': int, 'status': int, 'octet': int, 'sentTimestamp': float}]
        self.generalPacketQueue = []
        self.sysAdminPacketQueue = []

        # parameters for the ROI system, not ros interconnect.
        self.declare_parameter("timeout", 0.05)  # timeout in seconds
        self.declare_parameter("max_retries", 10)  # number of retries before abandoning packet
        self.declare_parameter("lost_to_disconnect", 1)
        self.declare_parameter(
            "network_address", "Null"
        )  # number of lost packets before reporting disconnect
        # generally if a packet is abandoned, then data is lost. This is a last resort to keep the system from hanging.
        # Adjust the timeout to stop lost packets, or improve network connectivity.

        # Create network sockets for general and sys admin packets
        self.generalNetworkSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.generalNetworkSocket.settimeout(self.get_parameter("timeout").value)

        self.sysAdminNetworkSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sysAdminNetworkSocket.settimeout(self.get_parameter("timeout").value)

        # generate arrays for tracking lost packets for connection state publishing
        self.octetConnected = [False] * 254  # connection state of octet
        self.octetLostPacketsSinceConnect = [0] * 254  # number of lost packets since last connect
        self.octetLostPacketsAccumulated = [0] * 254  # number of lost packets total

        self.parameterWatcherThread = threading.Thread(target=self.parameterWatcher, daemon=True)
        self.parameterWatcherThread.start()

        # Create threads for listening and transmitting packets, follows the queueing system
        self.netGeneralListenerThread = threading.Thread(
            target=self.netListener, daemon=True, args=(self.generalNetworkSocket,)
        )
        # self.netGeneralListenerThread.start()
        self.netSysAdminListenerThread = threading.Thread(
            target=self.netListener, daemon=True, args=(self.sysAdminNetworkSocket,)
        )
        # self.netSysAdminListenerThread.start()

        self.netTransmitterThread = threading.Thread(target=self.netTransmitter, daemon=True)
        # self.netTransmitterThread.start()

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

        # self.generalPacketQueue.append(
        # {
        # "packet": request.packet.data,
        # "uid": self.generateGeneralPacketUID(request.packet.data),
        # "status": 0,
        # "octet": request.packet.client_octet,
        # }
        # )

        self.sendToOctet(
            request.packet.client_octet, self.generalNetworkSocket, request.packet.data, GENERALPORT
        )

        # self.get_logger().info(f"General Packet Queue Length: {len(self.generalPacketQueue)}")

        return response

    def queueSysAdminPacketCallback(self, request, response):
        """Accepts a sys admin packet and queues it for sending

        Args:
            request (roi_ros.srv.QueueSerializedSysAdminPacket_Request): The request object
            response (roi_ros.srv.QueueSerializedSysAdminPacket_Response): The response object
        """
        uid = self.generateSysAdminPacketUID(request.packet.data)
        return self.sendPacket(self.sysAdminPacketQueue, uid, request, response)

    def sendPacket(self, queue, uid, request, response):
        response.success = True

        queue.append(
            {
                "packet": request.packet.data,
                "uid": uid,
                "status": 0,
                "octet": request.packet.client_octet,
            }
        )

        self.dynamicCreateTopics(request.packet.client_octet)

        return response

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
        try:
            network_address = self.get_parameter("network_address").value
            # self.get_logger().info(str(network_address))
            network_address = network_address.split(".")
            # self.get_logger().info(str(network_address))
            network_address.pop(3)
            # self.get_logger().info(str(network_address))
            # self.get_logger().info(str(type(network_address)))
            network_address.append(str(octet))
            # self.get_logger().info(str(network_address))
            # packet= 0b00000000_00000000_00000000_000000100_00000000_00000000_01000010100011000000000000000000.to_bytes(
            #    10, "big"
            # )

            bytestoSend = 0b00000000_00000000_00000000_00000000_00000000_00000000_00000001.to_bytes(
                7, "big"
            )
            # s.send(bytestoSend)
            # socket.sendto(bytestoSend, ("192.168.2.20", 57344))
            self.get_logger().info(str(len(self.generalPacketQueue)))

            # data = s.recv(64)

            # init pin 2 as output
            #               subdevice id,       action code,       checksum,       payload
            bytestoSend = 0b00000000_00000000_00000000_000000100_00000000_00000000_01000010100011000000000000000000.to_bytes(
                10, "big"
            )
            # s.send(bytestoSend)
            # socket.sendto(bytestoSend, ("192.168.2.20", 57344))

            network_address = ".".join(network_address)

            # socket.sendto(packet,(network_address, port))
            socket.sendto(packet, (network_address, port))

        except Exception as e:
            self.get_logger().error(f"Error: {e} at sending packet to octet {octet}")
        # self.get_logger().info(
        #     str(
        #         ".".join(
        #             self.get_parameter("network_address").value.split(".")[:-1] + [str(octet)]
        #         ),
        #     )
        # )

    def netListener(self, socket):
        """Listens for incoming packets on the network"""
        while rclpy.ok():

            try:
                data, addr = socket.recvfrom(ROI_MAX_PACKET_SIZE)
            except KeyboardInterrupt:
                self.get_logger().info("Keyboard interrupt, shutting down")
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e} at listening to network")
                continue
            if addr[0] == self.get_parameter("network_address").value:
                continue  # hi, we send a packet to ourself

            self.get_logger().info("net listener looping")
            data = [int.from_bytes(data[i]) for i in range(len(data))]  # Convert bytes to int

            if socket == self.generalNetworkSocket:
                queue = self.generalPacketQueue
                packet_type = "general"
                response_pub = self.octetResponsePublishers
                uid = self.generateGeneralPacketUID(data)
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
                else:
                    # mark octet as connected
                    self.octetConnected[int(addr[0].split(".")[3].strip())] = True
                    self.octetLostPacketsSinceConnect[int(addr[0].split(".")[3].strip())] = 0

                    # publish connection state
                    self.publishConnectionState(int(addr[0].split(".")[3].strip()))

                self.get_logger().info("received packet")

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
                queue = self.sysAdminPacketQueue
                packet_type = "sys admin"
                response_pub = self.sysAdminOctetResponsePublishers
                uid = self.generateSysAdminPacketUID(data)

            i = 0
            while i < len(queue):
                if queue[i]["uid"] == uid and addr[0].split(".")[3].strip() == queue[i]["octet"]:
                    queue.pop(i)
                    i = -1
                    break
                i += 1
            if i != -1:
                self.get_logger().info(
                    f"Received unwarranted {packet_type} response from {addr[0]}. Timeout: {self.get_parameter('timeout').value} may be too short."
                )
            else:
                # mark octet as connected
                self.octetConnected[int(addr[0].split(".")[3].strip())] = True
                self.octetLostPacketsSinceConnect[int(addr[0].split(".")[3].strip())] = 0

                # publish connection state
                self.publishConnectionState(int(addr[0].split(".")[3].strip()))

            try:
                response_pub[int(addr[0].split(".")[3].strip())].publish(
                    SerializedPacket(
                        data=data,
                        client_octet=int(addr[0].split(".")[3].strip()),
                        length=len(data),
                    )
                )
            except Exception as e:
                self.get_logger().error(f"Error: {e} at publishing {packet_type} response")

    def netTransmitter(
        self,
    ):
        """Transmits packets over the network"""
        while rclpy.ok():
            timeout = self.get_parameter("timeout").value
            retries = self.get_parameter("max_retries").value

            packets = [
                (self.generalPacketQueue, self.generalNetworkSocket, GENERALPORT),
                (self.sysAdminPacketQueue, self.sysAdminNetworkSocket, SYSADMINPORT),
            ]

            for packet_type in packets:
                queue, net_socket, port = packet_type
                if len(queue) > 0:
                    for packet in queue:
                        if packet["status"] == 0 or (
                            packet["status"] >= 1
                            and packet["status"] < retries
                            and time.time() - packet["sentTimestamp"] >= timeout
                        ):
                            self.sendToOctet(packet["octet"], net_socket, packet["packet"], port)
                            packet["status"] += 1
                            packet["sentTimeStamp"] = time.time()
                        else:
                            queue.remove(packet)

                            self.octetLostPacketsSinceConnect[packet["octet"]] += 1
                            self.octetLostPacketsAccumulated[packet["octet"]] += 1

                            if (
                                self.octetLostPacketsSinceConnect[packet["octet"]]
                                >= self.get_parameter("lost_to_disconnect").value
                            ):
                                self.octetConnected[packet["octet"]] = False

                            self.publishConnectionState(packet["octet"])

    def publishConnectionState(self, octet):
        """Publishes the connection state of an octet

        Args:
            octet (int): The octet to publish the connection state for
        """
        self.octetConnectionStatePublishers[octet].publish(
            ConnectionState(
                module_connected=self.octetConnected[octet],
                lost_packets_since_connect=self.octetLostPacketsSinceConnect[octet],
                lost_packets_accumulated=self.octetLostPacketsAccumulated[octet],
            )
        )

    def parameterWatcher(self):
        """Watches for the network address parameter to change
        Updates network sockets.
        """
        lastKnownNetworkAddress = self.get_parameter("network_address").value

        while rclpy.ok():
            if (
                self.get_parameter("network_address").value != lastKnownNetworkAddress
                and self.get_parameter("network_address").value != "Null"
            ):
                self.get_logger().info(
                    f"Network address changed from {lastKnownNetworkAddress} to {self.get_parameter('network_address').value}"
                )
                lastKnownNetworkAddress = self.get_parameter("network_address").value

                # close the sockets
                try:
                    self.generalNetworkSocket.close()
                    self.sysAdminNetworkSocket.close()
                except:
                    pass

                # create new sockets
                self.generalNetworkSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.generalNetworkSocket.settimeout(self.get_parameter("timeout").value)
                # self.generalNetworkSocket.bind(
                #    ("0.0.0.0", GENERALPORT))

                self.sysAdminNetworkSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sysAdminNetworkSocket.settimeout(self.get_parameter("timeout").value)
                # self.sysAdminNetworkSocket.bind(
                #    ("0.0.0.0", SYSADMINPORT)
                # )
                self.get_logger().info("Network address updated")
            time.sleep(0.5)

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


def main(args=None):
    rclpy.init(args=args)

    transport_agent = TransportAgent()

    rclpy.spin(transport_agent)

    transport_agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
