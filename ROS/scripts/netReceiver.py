import multiprocessing, socket, select


def readyToRead(socket, timeout):
    """blocks until timeout or data present

    Args:
        socket (UDP socket.socket): the rocket to check.
        timeout (float): The waiting time

    Returns:
        bool, true = data awaiting
    """

    Ready, _, _ = select.select([socket], [], [], timeout)
    return bool(Ready)


def netReceiver(rxQueue, timeout, port, dataSize):
    """A multiprocessing receiver thread for ROI. Not scalable, as it binds to the port.

    Args:
        rxQueue (multiprocessing.joinableQueue): The output queue from this process
        timeout (float): the socket timeout
        port (int): port to listen on
        dataSize(int): size of udp packets
    """
    try:
        rcvSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rcvSocket.settimeout(timeout)
        rcvSocket.bind(("0.0.0.0", port))

        while True:
            if not readyToRead(rcvSocket, 99):
                continue

            data, addr = rcvSocket.recvfrom(dataSize)

            octet = addr[0].split(".")[3]

            # {
            # "packet": request.packet.data,
            # "octet": request.packet.client_octet,
            # }

            entry = {"packet": data, "octet": octet}

            rxQueue.put(entry)

    finally:
        rcvSocket.close()
