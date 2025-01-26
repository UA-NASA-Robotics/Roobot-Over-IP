#include <Arduino.h>
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

uint8_t* generalBuffer(nullptr);  // Sharing a large buffer from the infrastructure in this main.cpp
ModuleInfrastructure* infraRef(
    nullptr);  // Reference to the infrastructure for withing handleGeneralPacket function

ModuleInfrastructure infra(
    10, 2, ***MODULE TYPE INDEX HERE***,
    ***General Packet Handler Function Here***);  // Create an instance of the infrastructure

void setup() {
    infra.init();  // Initialize the infrastructure (also defines Serial)

    infraRef = &infra;  // lets the handleGeneralPacket function access the infrastructure
    generalBuffer =
        &infra.generalBuffer[0];  // lets the handleGeneralPacket function access the buffer

    // Do any additional setup here

    infra.moduleStatusManager.notifyInitializedStatus();  // Notify the infrastructure that the
                                                          // module has been initialized.
}

ISR(TIMER1_OVF_vect) {
    // This ISR is called every 1.048 seconds by timer1 overflow
    infra.interruptNotification();  // Notify the infrastructure of the interrupt
}

void loop() {
    // Do any additional loop functions here

    infra.tick();  // Tick the infrastructure
}
