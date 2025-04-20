#include "macGen.h"

using namespace macGen;

// -- Public methods -- //
macAddressHelper::macAddressHelper(const uint8_t hostAddressOctet) {
    // Generate the MAC address using the host address octet and the seed values

    for (int i = 0; i < 6; i++) {
        _mac[i] = compileTime[i] ^ compileDate[i] ^
                  hostAddressOctet;  // XOR the seed values with the host address
                                     // octet to generate the MAC address
    }  // This should be a mac address randomized by compile time and by IP so that only one MAC
       // address is generated per IP address

    // There is the potential for a possible buffer overrun here, as compileTime and compileDate
    // length is not guaranteed to be 6 bytes. Currently not considered a problem as hashing heap
    // memory to make a random number not explicitly a vulnerability, but it is a potential issue.

    // We never operate at physical layer 2, so we only really worry about identifying by IP not
    // MAC address

    // set the 2nd bit of the first byte to 1 to indicate a locally administered MAC address
    // and set the least significant bit of the first byte to 0 to indicate unicast
    _mac[0] = (_mac[0] & 0xFE) | 0x02;
}

macAddressHelper::macAddressHelper() {
    // Generate the MAC address using the host address octet and the seed values

    for (int i = 0; i < 6; i++) {
        _mac[i] =
            compileTime[i] ^ compileDate[i] ^ 0xFF;  // XOR the seed values with the host address
                                                     // octet to generate the MAC address
    }  // This should be a mac address randomized by compile time and by IP so that only one MAC
       // address is generated per IP address

    // There is the potential for a possible buffer overrun here, as compileTime and compileDate
    // length is not guaranteed to be 6 bytes. Currently not considered a problem as hashing heap
    // memory to make a random number not explicitly a vulnerability, but it is a potential issue.

    // We never operate at physical layer 2, so we only really worry about identifying by IP not
    // MAC address

    // set the 2nd bit of the first byte to 1 to indicate a locally administered MAC address
    // and set the least significant bit of the first byte to 0 to indicate unicast
    _mac[0] = (_mac[0] & 0xFE) | 0x02;
}

macAddressHelper::~macAddressHelper() {
    // Destructor
    // No need to do anything here, as the MAC address is not dynamically allocated
    // and will be automatically freed when the object is destroyed
}

bool macAddressHelper::getMac(uint8_t* macBuffer) {
    for (int i = 0; i < 6; i++) {
        macBuffer[i] = this->_mac[i];
    }
    return true;
}