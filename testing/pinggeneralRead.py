import socket, time

# udp socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3)
s.bind(("192.168.2.133", 57344))
s.connect(("192.168.2.20", 57344))

# init pin 2 as output
#               subdevice id,       action code,       checksum,       payload
bytestoSend = 0b00000000_00000000_00000000_00000000_00000000_00000000_00000001.to_bytes(7, "big")
s.send(bytestoSend)

# data = s.recv(64)

# init pin 2 as output
#               subdevice id,       action code,       checksum,       payload
bytestoSend = 0b00000000_00000000_00000000_000000100_00000000_00000000_01000010100011000000000000000000.to_bytes(
    10, "big"
)
s.send(bytestoSend)

data = s.recv(64)

exit()
starttime = time.time()
i = 0
while i < 100:
    # send data
    # subdevice id, action code, checksum, payload
    bytestoSend = 0b00000000_00001010_00000000_00000011_00000000_00000001_00000000.to_bytes(
        7, "big"
    )
    s.send(bytestoSend)
    # receive data

    data = s.recv(60)

    print("received data: ", data)

    print("subdevice id: ", int.from_bytes(data[0:2], "big"))
    print("action code: ", int.from_bytes(data[2:4], "big"))
    print("checksum: ", int.from_bytes(data[4:6], "big"))
    print("payload: ", int.from_bytes(data[6:8], "big"))

    i += 1

endtime = time.time()
s.close()

print("done")
print("Time: ", endtime - starttime)
print("pps: ", i / (endtime - starttime))
