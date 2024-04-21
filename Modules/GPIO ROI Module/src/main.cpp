#include <Arduino.h>
#include <Ethernet.h>
#include <stdint.h>

#include "../../../lib/Packet.h"
#include "../../lib/macGen.h"

macGen::macAddressHelper macHelper;
uint8_t mac[6];
uint8_t IP[4] = {10, 49, 28, 231};  // IP address of the ROI module TO BE UPDATED LATER

void setup() {
    macHelper.getMac(mac);

    Ethernet.begin(mac, IP);
    server.begin();
    Serial.begin(9600);
    Serial.println("Server started");
}

void loop() {
    EthernetClient client = server.available();
    if (client) {
        Serial.println("Client connected");
        while (client.connected()) {
            if (client.available()) {
                Serial.println("Client available");
                uint8_t packetBuffer[100];
                client.read(packetBuffer, 100);
                ROIPackets::Packet packet;
                packet.importPacket(packetBuffer);
                Serial.println("Packet imported");
                uint8_t data[100];
                packet.getData(data);
                Serial.println("Data extracted");
                Serial.println((char *)data);
                client.write(data, 100);
                Serial.println("Data sent");
            }
        }
        client.stop();
        Serial.println("Client disconnected");
    }
}
