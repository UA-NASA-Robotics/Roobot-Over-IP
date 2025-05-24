#ifndef OctetSelector_H
#define OctetSelector_H

#ifndef DEBUG
#define DEBUG false
#endif

#include <stdint.h>
#ifdef __AVR__
#include <Arduino.h>
#else
// For non-AVR systems
#endif

namespace OctetSelectorConstants {
constexpr uint8_t OCTET_SELECT_CLOCK_DELAY =
    1;  // wait 1 ms after clocking the selector to read the octet
}

class OctetSelectorRev1 {
   protected:
#ifdef __AVR__
#ifdef __AVR_ATmega328PB__
    // Arduino Specific Functions

    /**
     * @brief Hardware specific function to read the octet from the hardware port
     *
     * @return bool
     */
    bool _readPortE();

    /**
     * @brief Hardware specific function to clock the selector from port e
     *
     */
    void _clockPortE(bool clockState);
#else
#endif
#endif

   public:
    OctetSelectorRev1();

    /**
     * @brief Sets up the octet selector during void setup()
     *
     */
    virtual void init();

    /**
     * @brief Reads the octet from hardware
     *
     * @return uint8_t
     */
    virtual uint8_t readOctet();
};

class OctetSelectorRev2 : public OctetSelectorRev1 {
   private:
   public:
    OctetSelectorRev2();

    /**
     * @brief Sets up the octet selector during void setup()
     *
     */
    void init();

    /**
     * @brief Reads the octet from hardware
     *
     * @return uint8_t
     */
    uint8_t readOctet();
};

class OctetSelectorRevNull : public OctetSelectorRev1 {
   private:
   public:
    OctetSelectorRevNull();

    /**
     * @brief Sets up the octet selector during void setup()
     *
     */
    void init() override;

    /**
     * @brief Reads the octet from hardware
     *
     * @return uint8_t
     */
    uint8_t readOctet() override;
};

#endif