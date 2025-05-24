import multiprocessing, socket, time


def sendToOctet(octet, socket, packet, port, networkAddress):
    """Sends a packet to a specific octet based on the networkAddress

    Args:
        octet (int): The octet to send the packet to
        socket (socket): The socket to use
        packet (byteArray): The packet data
        networkAddress (string): The network address of this computer.
    """
    try:
        network_address = networkAddress.split(".")
        network_address.pop(3)
        network_address.append(str(octet))

        socket.sendto(packet, (network_address, port))

    except Exception as e:
        print(f"Error: {e} at sending packet to octet {octet}")


def netTransmitter(jobQueue, networkAddress="0.0.0.0", sendTimeout=0.5, port=57344):
    """Transmits packets over the network. Made to run as a deamon thread. Spin a new one to change networkAddress.

    Args:
    jobQueue: The multiprocessing packet send queue
    networkAddress: the address as a string for this device.
    sendTimeout: the timeout for successful send.
    """
    try:
        generalNetworkSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        generalNetworkSocket.settimeout(sendTimeout)
        while True:
            # {
            # "packet": request.packet.data,
            # "uid": self.generateGeneralPacketUID(request.packet.data),
            # "status": 0,
            # "octet": request.packet.client_octet,
            # }
            packet: dict = jobQueue.get(block=True)

            sendToOctet(
                packet["octet"], generalNetworkSocket, packet["packet"], port, networkAddress
            )

    finally:
        generalNetworkSocket.close()
