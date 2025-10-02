import socket, time

# udp socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3)
s.bind(("192.168.2.133", 57664))
s.connect(("192.168.2.255", 57664))

starttime = time.time()
i = 0
while i < 10:
    # send data
    time.sleep(1)
    bytestoSend = 0b00000000000000001110011110100000000000000010000011100111.to_bytes(7, "big")
    s.send(bytestoSend)
    # receive data
    continue
    data = s.recv(1000)
    print("received data: ", data)
    metadata = data[0:2]  # 2 bytes
    print("metadata: ", int.from_bytes(metadata, "big"))
    data = data[2:]  # remove metadata
    host = data[0:1]  # 1 byte
    print("host: ", int.from_bytes(host, "big"))
    data = data[1:]  # remove host
    actioncode = data[0:2]  # 2 bytes
    print("actioncode: ", int.from_bytes(actioncode, "big"))
    data = data[2:]  # remove actioncode
    checksum = data[0:2]  # 2 bytes
    print("checksum: ", int.from_bytes(checksum, "big"))
    data = data[2:]  # remove checksum
    payload = data
    print("system status: ", int.from_bytes(payload[0:1], "big"))
    print("time hrs: ", int.from_bytes(payload[1:2], "big"))
    print("time mins: ", int.from_bytes(payload[2:3], "big"))
    print("time secs: ", int.from_bytes(payload[3:4], "big"))
    print("spply voltage: ", int.from_bytes(payload[4:6], "big"))
    print("mdule type: ", int.from_bytes(payload[6:7], "big"))
    print("chain host octet:", int.from_bytes(payload[7:8], "big"))
    print("MAC address: ", payload[8:14])

    i += 1

endtime = time.time()
s.close()

print("done")
print("Time: ", endtime - starttime)
print("pps: ", i / (endtime - starttime))
