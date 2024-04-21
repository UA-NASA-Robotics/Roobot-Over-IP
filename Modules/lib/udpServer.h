#ifndef tcpOverUDP_H
#define tcpOverUDP_H

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "macGen.h"

namespace ROIUDPServer {

class ROIUDPServer {
   protected:
    macGen::macAddressHelper macHelper;
    uint8_t mac[6];
    uint8_t IP[4];

    EthernetUDP general;
    EthernetUDP interrupt;
    EthernetUDP sysAdmin;

   private:
};

}  // namespace ROIUDPServer

#endif