from struct import pack
import multiprocessing, time, datetime


def generateGeneralPacketUID(packetDict):
    """Generates a UID for a general packet

    Args:
        packetArray (Array): The packet data

    Returns:
        int: The generated UID
    """
    return (
        (
            packetDict["packet"][0]
            + packetDict["packet"][1] * 7
            + packetDict["packet"][2] * 11
            + packetDict["packet"][3] * 13
            + int(packetDict["octet"]) * 13
        )
        * 17
        // 1024
    )


def generateSysAdminPacketUID(packetDict):
    """Generates a UID for a sys admin packet

    Args:
        packetarray (Array): The packet data

    Returns:
        int: The generated UID
    """
    return (
        (
            packetDict["packet"][2] * 7
            + packetDict["packet"][3] * 11
            + packetDict["packet"][4] * 13
            + int(packetDict["octet"]) * 15
        )
        * 19
        // 1024
    )


def reliabilityManager(
    incomingQueue,
    outgoingQueue,
    txQueue,
    rxQueue,
    connectionQueue,
    UIDFunc,
    resendTimeout,
    resendCount,
    lostToDisconnect,
):

    def importFromQueue(incomingQueue, txQueue, outstandingDict):
        """Import all incoming packets into outstanding zone and send to tx queue

        Args:
            incomingQueue (joinable queue): the incoming packets to manage
            txQueue (joinable queue): the to send queue
            outstandingDict (dict): the outstanding reliability connections dict
        """
        while not incomingQueue.empty():
            try:
                packetDict = incomingQueue.get(block=False)
                packetDict["timestamp"] = time.time()
                packetDict["resendCount"] = 0
                packetDict["multiplicity"] = 0
            except:
                # queue empty, stop
                return
            txQueue.put(packetDict)
            if UIDFunc(packetDict) in outstandingDict:
                # packet already in outstanding, increment resend count
                packetDict["multiplicity"] += 1
            else:
                outstandingDict[UIDFunc(packetDict)] = packetDict

    def exportFromQueue(
        outgoingQueue,
        rxQueue,
        connectionQueue,
        outstandingDict,
        octetConnected,
        octetLostPacketsSinceConnect,
        octetLostPacketsAccumulated,
    ):
        """Export all from RX queue and mark off outstanding packets

        Args:
            outgoingQueue (joinable queue): the outgoing packets to manage
            rxQueue (joinable queue): the received packets to manage
            outstandingDict (dict): the outstanding reliability connections dict
        """

        while not rxQueue.empty():
            try:
                packetDict = rxQueue.get(block=False)
            except:
                # empty, stop
                return

            outgoingQueue.put(packetDict)
            # packet was received, mark off outstanding

            UID = UIDFunc(packetDict)

            if UID in outstandingDict:
                if outstandingDict[UID]["multiplicity"] > 0:
                    # packet was resent, decrement multiplicity
                    outstandingDict[UID]["multiplicity"] -= 1
                else:
                    outstandingDict.pop(UID)

            octet = packetDict["octet"]

            octetConnected[octet] = True
            octetLostPacketsSinceConnect[octet] = 0

            connectionQueue.put(
                {
                    "octet": octet,
                    "lost_packets_since_connect": octetLostPacketsSinceConnect[octet],
                    "lost_packets_accumulated": octetLostPacketsAccumulated[octet],
                    "octet_connected": True,
                }
            )

        # generate arrays for tracking lost packets for connection state publishing

    octetConnected = [False] * 254  # connection state of octet
    octetLostPacketsSinceConnect = [0] * 254  # number of lost packets since last connect
    octetLostPacketsAccumulated = [0] * 254  # number of lost packets total

    # {
    # "packet": request.packet.data,
    # "uid": self.generateGeneralPacketUID(request.packet.data),
    # "timestamp": utc timestamp
    # "octet": request.packet.client_octet,
    # }

    outstandingPackets = {}

    while True:
        importFromQueue(incomingQueue, txQueue, outstandingPackets)
        exportFromQueue(
            outgoingQueue,
            rxQueue,
            connectionQueue,
            outstandingPackets,
            octetConnected,
            octetLostPacketsSinceConnect,
            octetLostPacketsAccumulated,
        )

        # resend packets that are outstanding
        for key in outstandingPackets.keys():
            if outstandingPackets[key]["timestamp"] + resendTimeout < time.time():
                # packet is old, resend
                outstandingPackets[key]["resendCount"] += 1
                if outstandingPackets[key]["resendCount"] > resendCount:
                    # packet has been resent too many times, remove from outstanding
                    outstandingPackets.pop(key)

                    octetLostPacketsSinceConnect[outstandingPackets[key]["octet"]] += 1
                    octetLostPacketsAccumulated[outstandingPackets[key]["octet"]] += 1
                    if (
                        octetLostPacketsSinceConnect[outstandingPackets[key]["octet"]]
                        >= lostToDisconnect
                        and octetConnected[outstandingPackets[key]["octet"]]
                    ):
                        # packets have been lost too many times, mark octet as disconnected
                        octetConnected[outstandingPackets[key]["octet"]] = False

                    connectionQueue.put(
                        {
                            "octet": outstandingPackets[key]["octet"],
                            "lost_packets_since_connect": octetLostPacketsSinceConnect[
                                outstandingPackets[key]["octet"]
                            ],
                            "lost_packets_accumulated": octetLostPacketsAccumulated[
                                outstandingPackets[key]["octet"]
                            ],
                            "octet_connected": octetConnected[outstandingPackets[key]["octet"]],
                        }
                    )
                else:
                    # packet is still valid, resend
                    txQueue.put(outstandingPackets[key])
                    outstandingPackets[key]["timestamp"] = time.time()

        if incomingQueue.empty() and rxQueue.empty():
            # no packets to process, sleep for a bit
            time.sleep(resendTimeout)
