#include "octetSelector.h"

OctetSelectorRev1::OctetSelectorRev1() {}

void OctetSelectorRev1::init() {
#ifdef __AVR__
#ifdef __328PB__
    // Set PortE pin 0 as output
    DDRE |= 0b0001;   // Set the first bit as output
    PORTE &= 0b1110;  // Set the first bit to output low

    // Set PortE pin 1 as input with no pull-up
    DDRE &= 0b1101;   // Set the second bit as input
    PORTE &= 0b1101;  // Set the second bit to input with no pull-up
#else

#endif
#endif
}

uint8_t OctetSelectorRev1::readOctet() {
#if DEBUG && defined(__AVR__)
    Serial.println("Reading Octet");
#endif

    uint8_t octet = 0;
#ifdef __AVR__
#ifdef __328PB__
    for (int i = 0; i < 8; i++) {
        delay(OctetSelectorConstants::OCTET_SELECT_CLOCK_DELAY);

        octet = (octet >> 1) + (_readPortE() << 7);  // Read the port and shift the octet
        // oppsie we are reading little endian

        _clockPortE(true);  // Clock the selector
        delay(OctetSelectorConstants::OCTET_SELECT_CLOCK_DELAY);
        _clockPortE(false);  // Clock the selector
    }
    // octet = (octet >> 1) + (readPortE() << 7);  // Read the last bit

#else

#endif
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
#ifdef __328PB__

bool OctetSelectorRev1::_readPortE() {
    // Read the port e 1 and return the value
    return PINE & 0b0010;
}

void OctetSelectorRev1::_clockPortE(bool clockState) {
    // Clock the selector on port e 0
    if (clockState) {
        PORTE |= 0b0001;
    } else {
        PORTE &= 0b1110;
    }
}

#else

#endif
#endif

OctetSelectorRev2::OctetSelectorRev2() {}

void OctetSelectorRev2::init() {
#ifdef __AVR__
#ifdef __328PB__
    // Set PortE pin 0 as output
    DDRE |= 0b0001;   // Set the first bit as output
    PORTE &= 0b1110;  // Set the first bit to output low

    // Set PortE pin 1 as input with no pull-up
    DDRE &= 0b1101;   // Set the second bit as input
    PORTE &= 0b1101;  // Set the second bit to input with no pull-up

    pinMode(A6, OUTPUT);     // Set the A6 pin as output
    digitalWrite(A6, HIGH);  // Set the A6 pin to output high, it is an active low pin
#else

#endif
#endif
}

uint8_t OctetSelectorRev2::readOctet() {
#if DEBUG && defined(__AVR__)
    Serial.println("Reading Octet");
#endif

    uint8_t octet = 0;
#ifdef __AVR__
#ifdef __328PB__
    digitalWrite(A6, LOW);  // Set the A6 pin to output low, it is an active low pin
    delay(OctetSelectorConstants::OCTET_SELECT_CLOCK_DELAY);
    digitalWrite(A6, HIGH);  // Set the A6 pin to output high, it is an active low pin

    _clockPortE(true);  // Clock the selector
    delay(OctetSelectorConstants::OCTET_SELECT_CLOCK_DELAY);
    _clockPortE(false);  // Clock the selector

    for (int i = 0; i < 8; i++) {
        delay(OctetSelectorConstants::OCTET_SELECT_CLOCK_DELAY);

        octet = (octet >> 1) + (_readPortE() << 7);  // Read the port and shift the octet
        // oppsie we are reading little endian

        _clockPortE(true);  // Clock the selector
        delay(OctetSelectorConstants::OCTET_SELECT_CLOCK_DELAY);
        _clockPortE(false);  // Clock the selector
    }
#else

#endif
#endif

#if DEBUG && defined(__AVR__)
    Serial.print("Octet: ");
    Serial.println(octet);
#endif

    octet = octet ? octet : 5;  // If the octet is 0, set it to 5 a default minimum value, we can
                                // never have 0 as an octet

    return octet;
}

OctetSelectorRevNull::OctetSelectorRevNull() {}

void OctetSelectorRevNull::init() {}

uint8_t OctetSelectorRevNull::readOctet() {
#if defined(__AVR__) && DEBUG
    Serial.println("Set Octet to static 5");
#endif
    return 5;
}
