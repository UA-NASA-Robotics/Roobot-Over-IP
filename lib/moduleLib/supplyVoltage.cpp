#include "supplyVoltage.h"

// Read the voltage of the battery the Arduino is currently running on (in millivolts)
int supplyVoltageReader::getVoltage(void) {
#if defined(__AVR__)                                            // If we are using an AVR board
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)  // For mega boards
    const long InternalReferenceVoltage =
        1115L;  // Adjust this value to your boards specific internal BG voltage x1000
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX5) | (1 << MUX4) | (1 << MUX3) |
            (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#else  // For 168/328 boards
    const long InternalReferenceVoltage =
        1091L;  // Adjust this value to your boards specific internal BG voltage x1000
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) |
            (0 << MUX0);
#endif
    ADCSRA |= _BV(ADSC);                    // Start a conversion
    while (((ADCSRA & (1 << ADSC)) != 0));  // Wait for it to complete
    int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) /
                  10L;    // Scale the value; calculates for straight line value
    return results * 10;  // convert from centivolts to millivolts
#else
    return 0;  // If we are not using an AVR board, return 0
#endif
}

uint16_t supplyVoltageReader::getAccurateVCC() {  // The first reading is always wrong, so we take a
                                                  // second reading
    supplyVoltageReader::getVoltage();  // Get the voltage, but throw it away. First reading is
                                        // always wrong
    return supplyVoltageReader::getVoltage();  // Get the voltage again in millivolts. Return the
                                               // value and cast it to a uint16_t
}
