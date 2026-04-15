#ifndef OctetSelector_H
#define OctetSelector_H

#include <stdint.h>

#include "debug.h"
#include <Arduino.h>


namespace OctetSelectorConstants {
constexpr uint8_t OCTET_SELECT_CLOCK_DELAY =
    1;  // wait 1 ms after clocking the selector to read the octet
}


// TODO: Reformat pre-processer directives to be cleaner. Requires modifications of infrastructure
class OctetSelectorRev1 {
   #ifdef __AVR__
   #ifdef __AVR_ATmega328PB__
    protected:

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
   #ifdef __AVR__
   #ifdef __AVR_ATmega328PB__
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
    #endif
    #endif
};

class OctetSelectorRevNull : public OctetSelectorRev1 {
   #ifdef __AVR__
   #ifdef __AVR_ATmega328PB__
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
    #endif
    #endif
};


class OctetSelectorRev3 : public OctetSelectorRev1 { // Inherits from Rev1 to be of same type as other selectors
    #if ODRIVE_MODULE_REV == 3
    private:
    public:
    OctetSelectorRev3();

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
    #endif
};



#endif