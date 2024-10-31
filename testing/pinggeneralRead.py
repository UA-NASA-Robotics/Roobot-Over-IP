import socket, time

# udp socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3)
s.bind(("10.0.0.100", 57344))
s.connect(("10.0.0.231", 57344))

# init pin 2 as output
# subdevice id, action code, checksum, payload
bytestoSend = 0b00000000_00000011_00000000_00000110_00000000_00000001_01000010010010000000000000000000.to_bytes(
    10, "big"
)
s.send(bytestoSend)

data = s.recv(60)

time.sleep(100)

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
