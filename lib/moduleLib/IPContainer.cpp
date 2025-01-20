#include "IPContainer.h"

IPContainer::IPContainer(uint8_t networkHighByte, uint8_t networkHighMidByte,
                         uint8_t networkLowMidByte, uint8_t networkLowByte) {
    addressArray[0] = networkHighByte;
    addressArray[1] = networkHighMidByte;
    addressArray[2] = networkLowMidByte;
    addressArray[3] = networkLowByte;

    networkAddress = IPAddress(addressArray[0], addressArray[1], addressArray[2], addressArray[3]);
}

void IPContainer::updateIP(uint8_t networkHighByte, uint8_t networkHighMidByte,
                           uint8_t networkLowMidByte, uint8_t networkLowByte) {
    addressArray[0] = networkHighByte;
    addressArray[1] = networkHighMidByte;
    addressArray[2] = networkLowMidByte;
    addressArray[3] = networkLowByte;

    networkAddress = IPAddress(addressArray[0], addressArray[1], addressArray[2], addressArray[3]);
}

void IPContainer::updateIP(uint8_t hostAddress) {
    addressArray[3] = hostAddress;
    networkAddress = IPAddress(addressArray[0], addressArray[1], addressArray[2], addressArray[3]);
}