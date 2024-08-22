#include <Arduino.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#include <stdint.h>

// Define the default debug mode for the ROI module
#ifndef DEBUG
#define DEBUG false
#endif
// Modify debug mode in "PlatformIO.ini" file, NOT here

#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/moduleLib/chainNeighborManager.h"
#include "../../../lib/moduleLib/macGen.h"
#include "../../../lib/moduleLib/statusManager.h"
#include "../../../lib/moduleLib/sysAdminHandler.h"

using namespace GeneralGPIOConstants;  // Import the constants from the GeneralGPIOConstants
                                       // namespace as we will be using them in this file

// Hardware CONSTANTS
const uint8_t WIZ5500_CS_PIN = 10;  // Chip select pin for WIZ5500 module

macGen::macAddressHelper macHelper;
uint8_t mac[6];
uint8_t IPArray[4] = {10, 0, 0, 231};  // IP address of the ROI module TO BE UPDATED LATER
IPAddress IP(IPArray[0], IPArray[1], IPArray[2],
             IPArray[3]);  // Create an IP address instance for UDP

statusManager::statusManager
    moduleStatusManager;  // Create a status manager instance (manages the status of the ROI module)

// Create a UDP instances for each type of packet on the ROI module
EthernetUDP General;
EthernetUDP Interrupt;
EthernetUDP SysAdmin;

uint8_t generalBuffer[ROIConstants::ROIMAXPACKETSIZE];  // Buffer for packet import and export

chainNeighborManager::chainNeighborManager moduleChainManager(
    moduleTypesConstants::GeneralGPIO, IPArray, IPArray[3], moduleStatusManager, SysAdmin,
    generalBuffer);  // Create a chainNeighborManager instance

sysAdminHandler::sysAdminHandler moduleSysAdminHandler(
    moduleTypesConstants::GeneralGPIO, moduleStatusManager, moduleChainManager,
    generalBuffer);  // Create a sysAdminHandler instance

uint8_t subDeviceIDState[COUNT] = {
    INPUT_MODE};  // The state of each pin on the ROI module (Used for output safety check)

void setup() {
    // ISR for Chain Discovery setup
    TCCR1A = 0;  // set entire TIMER1 to zero, and initialize the timer1 registers
    TCCR1B = 0;

    TCCR1B |= 0b00000100;  // set the timer1 prescaler to 256. IE 16MHz/256 = 62500Hz. Timer1
                           // overflows at 65535, so 65535/62500 = 1.048 seconds

    TIMSK1 |= 0b00000001;  // enable timer1 overflow interrupt
    // End of ISR setup

    macHelper.getMac(
        mac);  // Get the MAC address from the EEPROM, or generate one if it doesn't exist
    moduleSysAdminHandler.setMAC(mac);  // Set the MAC address in the sysAdminHandler

    Ethernet.init(WIZ5500_CS_PIN);  // Initialize the Ethernet module SPI interface
    Ethernet.begin(mac, IP);        // Initialize the Ethernet module with the MAC and IP addresses

    w5500.setRetransmissionCount(1);  // Set the retransmission count to 1, ie 2 attempts
    w5500.setRetransmissionTime(2);   // Set the retransmission time to 2, ie 2 milliseconds

#if DEBUG
    Serial.begin(9600);  // Initialize the serial port for debugging
#endif

    delay(100);  // Wait for devices to initialize

    if (w5500.readPHYCFGR() && 0x01 == 0) {  // Check if the link status is connected
#if DEBUG
        Serial.println(F("Ethernet not connected."));
#endif
        while (w5500.readPHYCFGR() && 0x01 == 0) {
            delay(100);  // Wait for the Ethernet cable to be connected
        }
#if DEBUG
        Serial.println(F("Ethernet connected. Resuming operation."));
#endif
    }

    General.begin(ROIConstants::ROIGENERALPORT);     // Initialize the general UDP instance
    Interrupt.begin(ROIConstants::ROIINTERUPTPORT);  // Initialize the interrupt UDP instance
    SysAdmin.begin(ROIConstants::ROISYSADMINPORT);   // Initialize the sysAdmin UDP instance

    delay(500);  // Wait for devices to initialize

    moduleStatusManager.notifyInitializedStatus();  // Notify the status manager that the module is
    // initialized and ready for operation

#if DEBUG
    Serial.println(F("ROI Module is ready for operation."));
#endif
}

ISR(TIMER1_OVF_vect) {
    // This ISR is called every 1.048 seconds by timer1 overflow
    moduleChainManager.notifyDoDiscovery();  // Notify the chain manager to do discovery on the next
                                             // void loop cycle
}

void (*resetFunction)(void) = 0;  // declare reset function @ address 0

// Function to set the mode of a pin
//@param subDeviceID The subdevice ID of the pin to set the mode of. See module codec
//@param mode The mode to set the pin to
//@return True if the mode was set successfully, false otherwise
bool setPinMode(uint16_t subDeviceID, uint16_t mode) {
    if (subDeviceID == 8 || subDeviceID == 9 || subDeviceID > 17 || mode > OUTPUT_MODE) {
        return false;
    }
    uint8_t pin = subDeviceIDLookup[subDeviceID];  // Get the pin number from the subdevice ID
    switch (mode) {                                // Set the mode of the pin
        case INPUT_MODE:
            pinMode(pin, INPUT);
            break;
        case INPUT_PULLUP_MODE:
            pinMode(pin, INPUT_PULLUP);
            break;
        case OUTPUT_MODE:
            pinMode(pin, OUTPUT);
            break;
    };
    subDeviceIDState[subDeviceID] = mode;  // Update the state of the pin in the state array
    return true;
}

// Function to set the output of a pin
//@param subDeviceID The subdevice ID of the pin to set the output of. See module codec
//@param output The output to set the pin to (0-1)
bool setOutput(uint16_t subDeviceID, uint16_t output_state) {
    if (subDeviceID > 15 || subDeviceID == 8 || subDeviceID == 9 ||
        output_state > 1) {  // Check if the subdevice ID and output state are valid
        return false;
    }
    if (subDeviceIDState[subDeviceID] != OUTPUT_MODE) {  // Check if the pin is set to output mode
        return false;
    }
    uint8_t pin = subDeviceIDLookup[subDeviceID];  // Get the pin number from the subdevice ID
    digitalWrite(pin, output_state);               // Set the output of the pin
    return true;
}

// Function to read the value of a pin
//@param subDeviceID The subdevice ID of the pin to read the value of. See module codec
//@return The value of the pin, digital if subdevice ID is 0-7, analog if subdevice ID is 10-17
bool read(uint16_t subDeviceID, uint8_t* readBuffer) {
    if (subDeviceID > 17 || subDeviceID == 8 ||
        subDeviceID == 9) {  // Check if the subdevice ID is valid
        return false;
    }
    uint8_t pin = subDeviceIDLookup[subDeviceID];  // Get the pin number from the subdevice ID
    if (subDeviceID < 8) {                         // Read the digital value of the pin
        readBuffer[0] = digitalRead(pin);
    } else {  // Read the analog value of the pin
        uint16_t value = analogRead(pin);
        readBuffer[0] = highByte(value);
        readBuffer[1] = lowByte(value);
    }
    return true;
}

// Function to handle a general packet
//@param packet The packet to handle
ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet packet) {
    uint16_t action = packet.getActionCode();        // Get the action code from the packet
    uint16_t subDeviceID = packet.getSubDeviceID();  // Get the subdevice ID from the packet
    packet.getData(generalBuffer,
                   ROIConstants::ROIMAXPACKETPAYLOAD);  // Get the payload from the packet

    ROIPackets::Packet replyPacket = packet.swapReply();  // Create a reply packet

    switch (action) {
        case SET_PIN_MODE:
            uint8_t modeSet[1];
            modeSet[0] = setPinMode(subDeviceID, generalBuffer[0]);  // Set the mode of the pin

            replyPacket.setData(modeSet, 1);  // Set the mode of the pin

            moduleStatusManager
                .notifySystemConfigured();  // Notify the status manager that the system
            // has been configured, exits the blank state
            break;
        case SET_OUTPUT:
            uint8_t outputSet[1];
            outputSet[0] = setOutput(subDeviceID, generalBuffer[0]);  // Set the output of the pin

            replyPacket.setData(outputSet, 1);  // Set the output of the pin
            break;
        case READ:
            uint8_t readBuffer[2];
            read(subDeviceID, readBuffer);  // Read the value of the pin

            replyPacket.setData(readBuffer, 2);  // Set the value of the pin
            break;
    };

    return replyPacket;  // Return the reply packet
}

void loop() {
    // Check for connection status
    if (w5500.readPHYCFGR() && 0x01 == 0) {
#if DEBUG
        Serial.println(F("Ethernet cable is not connected. Reinitalizing."));
        delay(1000);  // delay for 1 second for serial to print
#endif
        resetFunction();  // The reset function is called to restart the module
        // The program will not fully resume operation until the Ethernet cable is connected
    }
    // Check for a general packet
    int generalPacketSize = General.parsePacket();
    if (generalPacketSize) {
        IPAddress remote = General.remoteIP();           // Get the remote IP address
        General.read(generalBuffer, generalPacketSize);  // Read the general packet
        ROIPackets::Packet generalPacket(IPArray[3],
                                         remote[3]);  // Create a general packet from the buffer
        generalPacket.importPacket(
            generalBuffer,
            ROIConstants::ROIMAXPACKETSIZE);  // Import the general packet from the buffer

        ROIPackets::Packet replyPacket =
            handleGeneralPacket(generalPacket);  // Handle the general packet

        replyPacket.exportPacket(
            generalBuffer,
            ROIConstants::ROIMAXPACKETSIZE);  // Export the reply packet to the buffer
        General.beginPacket(remote, ROIConstants::ROIGENERALPORT);  // Begin the reply packet
        General.write(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);
        General.endPacket();  // Send the reply packet
    }

    // Check for an interrupt packet
    // todo

    // Check for a sysAdmin packet
    int sysAdminPacketSize = SysAdmin.parsePacket();
    if (sysAdminPacketSize) {
        IPAddress remote = SysAdmin.remoteIP();            // Get the remote IP address
        SysAdmin.read(generalBuffer, sysAdminPacketSize);  // Read the general packet
        ROIPackets::sysAdminPacket sysAdminPacket(
            IPArray[3],
            remote[3]);  // Create a general packet from the buffer
        bool success = sysAdminPacket.importPacket(
            generalBuffer,
            ROIConstants::ROIMAXPACKETSIZE);  // Import the general packet from the buffer
        if (!success) {
            // Serial.println(F("Failed to import packet"));
            //-------------------------------return;  // Skip the rest of the loop if the packet
            //  failed to import
        }

        ROIPackets::sysAdminPacket replyPacket = moduleSysAdminHandler.handleSysAdminPacket(
            sysAdminPacket);  // Handle the sysAdmin packet

        if (replyPacket.getActionCode() == sysAdminConstants::BLANK) {
#if DEBUG
            Serial.println(F("Blank packet received, no reply needed"));
#endif
            return;
        }

        replyPacket.exportPacket(
            generalBuffer,
            ROIConstants::ROIMAXPACKETSIZE);  // Export the reply packet to the buffer

        uint8_t sent = 0;
        while (sent < 10) {
            SysAdmin.beginPacket(remote, ROIConstants::ROISYSADMINPORT);  // Begin the reply packet
            SysAdmin.write(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);
            if (SysAdmin.endPacket()) {  // if the packet was sent successfully quit the loop
                sent = 10;
            }  // Send the reply packet
            sent++;
        }
    }

    moduleChainManager.discoverChain();  // Discover the chain neighbors (Does nothing if not
                                         // activated by ISR)
}
