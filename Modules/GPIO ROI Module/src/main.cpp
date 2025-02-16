#include <stdint.h>

// Define the default debug mode for the ROI module
#ifndef DEBUG
#define DEBUG false
#endif
// Modify debug mode in "PlatformIO.ini" file, NOT here

#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/floatCast.h"
#include "../../../lib/moduleLib/infrastructure.h"

using namespace GeneralGPIOConstants;

uint8_t subDeviceIDState[COUNT] = {
    INPUT_MODE};  // The state of each pin on the ROI module (Used for output safety check)

uint8_t* generalBuffer(nullptr);          // Buffer for general packets
ModuleInfrastructure* infraRef(nullptr);  // Reference to the module infrastructure

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
                   ROIConstants::ROI_MAX_PACKET_PAYLOAD);  // Get the payload from the packet

    ROIPackets::Packet replyPacket = packet.swapReply();  // Create a reply packet

    switch (action) {
        case SET_PIN_MODE:
            uint8_t modeSet[1];
            modeSet[0] = setPinMode(subDeviceID, generalBuffer[0]);  // Set the mode of the pin

            replyPacket.setData(modeSet, 1);  // Set the mode of the pin

            infraRef->moduleStatusManager
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

ModuleInfrastructure infra(10, 2, moduleTypesConstants::GENERAL_GPIO, handleGeneralPacket);

void setup() {
    infra.init();
    generalBuffer = &infra.generalBuffer[0];
    infraRef = &infra;

    infra.moduleStatusManager.notifyInitializedStatus();
}

ISR(TIMER1_OVF_vect) {
    // This ISR is called every 1.048 seconds by timer1 overflow
    infra.interruptNotification();
}

void loop() { infra.tick(); }
