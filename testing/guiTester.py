try:
    import guizero
except:
    print("guizero not found")
    print("Please install guizero using the command 'python 3 -m pip install guizero'")
    exit(1)

import threading
import socket
import time

sendQueue = []
recvQueue = []
info = {
    "target": "192.168.2.5",
    "port": 57344,
    "self": "192.168.2.100",
    "connected": False,
    "socketSuccess": False,
}

sthread = None
lThread = None
cThread = None


def sendrcvThread(sendQueue, recvQueue, info):
    # udp socket
    info["socketSuccess"] = True
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(3)  # timeout of 3 seconds
        s.bind((info["self"], info["port"]))
        s.connect((info["target"], info["port"]))
    except:
        print("socket error")
        info["socketSuccess"] = False

    while True:
        if not info["connected"]:
            s.close()
            info["socketSuccess"] = False
            break
        if len(sendQueue) > 0:
            s.send(sendQueue.pop(0))
        try:
            data = s.recv(64)
            recvQueue.append(data)
        except:
            print("timeout")
        time.sleep(0.25)


def toggleConnect():
    if info["connected"]:
        info["connected"] = False
    else:
        info["connected"] = True
        sthread = threading.Thread(target=sendrcvThread, args=(sendQueue, recvQueue, info))
        sthread.start()
        lThread = threading.Thread(target=listboxThread, args=(recvQueue, recvListbox, info))
        lThread.start()
        cThread = threading.Thread(target=connectedThread, args=(info,))
        cThread.start()

    info["target"] = targetIP.value
    info["port"] = int(port.value)
    info["self"] = selfIP.value


def stuffToByteArray():
    if int(subdeviceID.value) > 65535 or int(subdeviceID.value) < 0:
        app.warn("Subdevice ID out of range", "Subdevice ID must be between 0 and 65535")
        return False
    if int(actionCode.value) > 65535 or int(actionCode.value) < 0:
        app.warn("Action Code out of range", "Action Code must be between 0 and 65535")
        return False
    if len(data.value) % 8 != 0:
        app.warn("Data length not a multiple of 8", "Data length must be a multiple of 8")
        return False

    return (
        int(subdeviceID.value).to_bytes(2, "big")
        + int(actionCode.value).to_bytes(2, "big")
        + int(data.value, 2).to_bytes(len(data.value) // 8, "big")
    )


def send():
    data = stuffToByteArray()
    if data:
        sendQueue.append(data)


def listboxThread(recvQueue, recvListbox, info):
    while info["connected"]:
        if len(recvQueue) > 0:
            recvListbox.append(recvQueue.pop(0))
        time.sleep(0.25)

    recvListbox.clear()


def connectedThread(info):
    while info["connected"]:
        if not info["socketSuccess"] and isConnected.value:
            isConnected.toggle()
        elif info["socketSuccess"] and not isConnected.value:
            isConnected.toggle()

        time.sleep(0.5)

    if isConnected.value:
        isConnected.toggle()


if __name__ == "__main__":

    app = guizero.App(title="General Command Interface", width=500, height=600, layout="grid")
    portLabel = guizero.Text(app, text="Port:", grid=[0, 0])
    port = guizero.TextBox(app, text="57734", grid=[1, 0], width=30)
    selfIPLabel = guizero.Text(app, text="Self IP:", grid=[0, 1])
    selfIP = guizero.TextBox(app, text="192.168.2.100", grid=[1, 1], width=30)
    targetIPLabel = guizero.Text(app, text="Target IP:", grid=[0, 2])
    targetIP = guizero.TextBox(app, text="192.168.2.5", grid=[1, 2], width=30)
    isConnected = guizero.CheckBox(app, text="Connected", grid=[0, 3], enabled=False)
    connectButton = guizero.PushButton(
        app, text="Toggle Connect", grid=[1, 3], command=toggleConnect
    )

    subdeviceIDLabel = guizero.Text(app, text="Subdevice ID (int):", grid=[0, 4])
    subdeviceID = guizero.TextBox(app, text="0", grid=[1, 4])
    actionCodeLabel = guizero.Text(app, text="Action Code (int):", grid=[0, 5])
    actionCode = guizero.TextBox(app, text="0", grid=[1, 5])

    dataLabel = guizero.Text(app, text="Data (bit string):", grid=[0, 6])
    data = guizero.TextBox(app, text="0", grid=[1, 6], width=50)
    sendButton = guizero.PushButton(app, text="Send", grid=[1, 7], command=send)

    recvListbox = guizero.ListBox(
        app, items=[], grid=[0, 8, 2, 1], width=400, height=300, scrollbar=True
    )

    app.display()
    info["connected"] = False
    sthread.join()
    lThread.join()
    cThread.join()
    print("done")
    exit()
