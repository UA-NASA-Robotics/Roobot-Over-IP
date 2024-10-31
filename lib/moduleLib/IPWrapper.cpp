#include "IPWrapper.h"

IPWrapper::IPWrapper(OctetSelectorRev1* selector, uint8_t networkHighByte,
                     uint8_t networkHighMidByte, uint8_t networkLowMidByte) {
    octetSelector = selector;
    addressArray[0] = networkHighByte;
    addressArray[1] = networkHighMidByte;
    addressArray[2] = networkLowMidByte;
}

void IPWrapper::init() {
    addressArray[3] = octetSelector->readOctet();

    networkAddress = IPAddress(addressArray[0], addressArray[1], addressArray[2], addressArray[3]);
}