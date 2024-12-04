#include "octetSelector.h"

OctetSelectorRev1::OctetSelectorRev1() {}

void OctetSelectorRev1::init() {
#ifdef __AVR__
    // Set PortE0 as output
    DDRE &= ~(1 << DDE0);
    PORTE |= (1 << PORTE0);

    // Set PortE1 as Input
    DDRE &= ~(1 << DDE1);
    PORTE |= (1 << PORTE1);
#endif
}

uint8_t OctetSelectorRev1::readOctet() {
#if DEBUG && defined(__AVR__)
    Serial.println("Reading Octet");
#endif

    uint8_t octet = 0;
#ifdef __AVR__
    for (int i = 0; i < 8; i++) {
        delay(OctetSelectorConstants::clockDelay);

        octet = octet << 1 + readPortE();  // Read the port and shift the octet

        clockPortE(true);  // Clock the selector
        delay(OctetSelectorConstants::clockDelay);
        clockPortE(false);  // Clock the selector
    }
#endif

#if DEBUG && defined(__AVR__)
    Serial.print("Octet: ");
    Serial.println(octet);
#endif

    octet = octet ? octet : 5;  // If the octet is 0, set it to 5 a default minimum value, we can
                                // never have 0 as an octet

    return octet;
}

#ifdef __AVR__  // Arduino Specific Functions

bool OctetSelectorRev1::readPortE() {
    // Read the port e 1 and return the value
    return PINE & (1 << PINE1);
}

void OctetSelectorRev1::clockPortE(bool clockState) {
    // Clock the selector on port e 0
    if (clockState) {
        PORTE |= (1 << PORTE0);
    } else {
        PORTE &= ~(1 << PORTE0);
    }
}

#endif