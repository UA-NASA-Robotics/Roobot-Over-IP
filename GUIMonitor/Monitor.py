import socket, time, multiprocessing, select

try:
    import guizero
except ImportError:
    print("guizero not installed, please install guizero to run this script")
    exit(1)

DELAYTIME = 1


def get_host_ip():
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


def getStatus(delay, dataQueue):
    """Runs a socket connection to get the status of the system, pings the system and returns the status of the system

    Args:
        delay (int): delay in seconds between each status check
    """
    # udp socket

    ip = get_host_ip()
    print(ip)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(3)
    s.bind((ip, 57664))  # bind to the local ip address
    # s.connect((ip.replace(ip.split(".")[-1], "255"), 57664))  # connect to the local anycast address
    bytestoSend = 0b0000000000010001_11100111_10100000000000000010000011100111.to_bytes(7, "big")

    while True:
        s.sendto(bytestoSend, (ip.replace(ip.split(".")[-1], "255"), 57664))
        while True:
            try:
                data, addr = s.recvfrom(60)
                if addr[0] == ip:
                    continue

                metadata = data[0:2]  # 2 bytes
                data = data[2:]  # remove metadata
                host = data[0:1]  # 1 byte
                data = data[1:]  # remove host
                actioncode = data[0:2]  # 2 bytes
                data = data[2:]  # remove actioncode
                checksum = data[0:2]  # 2 bytes
                data = data[2:]  # remove checksum
                payload = data
                relaventData = {
                    "system status": int.from_bytes(payload[0:1], "big"),
                    "time hrs": int.from_bytes(payload[1:2], "big"),
                    "time mins": int.from_bytes(payload[2:3], "big"),
                    "time secs": int.from_bytes(payload[3:4], "big"),
                    "spply voltage": int.from_bytes(payload[4:6], "big"),
                    "mdule type": int.from_bytes(payload[6:7], "big"),
                    "chain host octet": int.from_bytes(payload[7:8], "big"),
                    "MAC address": payload[8:14],
                }
                dataQueue.put(relaventData)
            except Exception as e:
                print(f"Error: {e}")
            break
        time.sleep(delay)


def updateData():
    global dataPool

    copyPool = dataPool.copy()

    while not dataQueue.empty():
        for i in range(len(copyPool)):
            if copyPool[i]["MAC address"] == dataQueue.get()["MAC address"]:
                copyPool[i] = dataQueue.get()
                break
        else:
            copyPool.append(dataQueue.get())

    for i in range(len(copyPool)):
        copy = True
        for j in range(len(dataPool)):
            if copyPool[i]["MAC address"] == dataPool[j]["MAC address"]:
                copy = False
                break

        if copy:
            dataPool.append(copyPool[i])


def renderConsole():
    width = int(canvas._width)
    height = int(canvas._height)

    length = len(dataPool)

    canvas.clear()

    for i in range(length):
        hi = i % 4
        vi = i // 4
        canvas.rectangle(
            hi * width / 4,
            vi * height / 4,
            (hi + 1) * width / 4,
            (vi + 1) * height / 4,
            color="grey",
        )
        canvas.oval(
            hi * width / 4,
            vi * height / 4,
            hi * width / 4 + width / 25,
            vi * height / 4 + width / 25,
            color=(
                "green"
                if dataPool[i]["system status"] == 1
                else (
                    "yellow"
                    if dataPool[i]["system status"] <= 3 and dataPool[i]["system status"] >= 1
                    else "red" if dataPool[i]["system status"] < 6 else "blue"
                )
            ),
            outline=True,
        )

        canvas.text(
            hi * width / 4,
            vi * height / 4 + width / 20,
            f"MAC: {dataPool[i]['MAC address']}",
            color="white",
            size=10,
        )

        canvas.text(
            hi * width / 4 + width / 20,
            vi * height / 4,
            "GeneralGPIO" if dataPool[i]["mdule type"] == 3 else "OtherModule",
            color="white",
            size=10,
        )

        canvas.text(
            hi * width / 4,
            vi * height / 4 + width / 15,
            f"Voltage: {dataPool[i]['spply voltage']}",
            color="white",
            size=10,
        )

        canvas.text(
            hi * width / 4,
            vi * height / 4 + width / 10,
            f"Time: {dataPool[i]['time hrs']}:{dataPool[i]['time mins']}:{dataPool[i]['time secs']}",
            color="white",
            size=10,
        )


if __name__ == "__main__":
    dataQueue = multiprocessing.Queue()

    pingProcess = multiprocessing.Process(
        target=getStatus, args=(DELAYTIME, dataQueue), daemon=True
    )
    pingProcess.start()

    dataPool = []

    app = guizero.App(title="Monitor", width=800, height=600)

    canvas = guizero.Drawing(app, width="800", height="400")
    canvas.bg = "black"

    app.repeat(500, updateData)
    app.repeat(500, renderConsole)

    app.display()
