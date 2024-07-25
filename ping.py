import socket, time

# udp socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3)
s.bind(("10.0.0.19", 57664))
s.connect(("10.0.0.231", 57664))

starttime = time.time()
i = 0
while i < 1:
    # send data
    bytestoSend = 0b00000000000000001110011100100000000000000010000011100111.to_bytes(7, "big")
    s.send(bytestoSend)
    # receive data

    data = s.recv(60)

    i += 1

endtime = time.time()
s.close()

print("done")
print("Time: ", endtime - starttime)
print("pps: ", 1000 / (endtime - starttime))
