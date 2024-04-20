#include <Arduino.h>
#include <Ethernet2.h>
#include <stdint.h>

#include "../../../lib/Packet.h"
#include "../../lib/macGen.h"

macGen::macAddressHelper macHelper;
uint8_t mac[6];

void setup() { macHelper.getMac(mac); }

void loop() {}
