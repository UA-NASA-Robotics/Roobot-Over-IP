#include <Arduino.h>
#include <ODriveUART.h>
#include <SoftwareSerial.h>
#include <stdint.h>

// Define the default debug mode for the ROI module
#ifndef DEBUG
#define DEBUG false
#endif
// Modify debug mode in "PlatformIO.ini" file, NOT here

#ifndef USE_ROI_WATCHDOG
#define USE_ROI_WATCHDOG false
#endif

#include "../../../lib/Packet.h"
#include "../../../lib/floatCast.h"
#include "../../../lib/moduleLib/infrastructure.h"
#include "oDriveContainer.h"
#include "oDriveController.h"
#include "oDriveError.h"

uint8_t* generalBuffer(nullptr);  // Sharing a large buffer from the infrastructure in this main.cpp
ModuleInfrastructure* infraRef(
    nullptr);  // Reference to the infrastructure for withing handleGeneralPacket function

ODriveController controller1(
    8, 7, 115200, infraRef->moduleStatusManager);  // Create an instance of the ODriveController

ODriveContainer<1> oDriveContainer;  // Create an instance of the ODriveContainer

ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet packet) {
    return oDriveContainer.handleGeneralPacket(packet);
}

void staticPauseCallback() { oDriveContainer.pause(); }

void staticResumeCallback() { oDriveContainer.resume(); }

ModuleInfrastructure infra(10, 2, moduleTypesConstants::O_DRIVE,
                           handleGeneralPacket);  // Create an instance of the infrastructure

void setup() {
    infra.init();  // Initialize the infrastructure (also defines Serial)

    infraRef = &infra;  // lets the handleGeneralPacket function access the infrastructure
    generalBuffer =
        &infra.generalBuffer[0];  // lets the handleGeneralPacket function access the buffer

    oDriveContainer.append(controller1);  // Append the controller to the container

    oDriveContainer.init();  // Initialize the container

#if USE_ROI_WATCHDOG
    infra.moduleStatusManager.setDisconnectCallback(staticPauseCallback);  // Set the pause
    callback infra.moduleStatusManager.setReconnectCallback(
        staticResumeCallback);  // Set the resume callback
#endif

    infra.moduleStatusManager.notifyInitializedStatus();  // Notify the infrastructure that the
                                                          // module has been initialized.
}

ISR(TIMER1_OVF_vect) {
    // This ISR is called every 1.048 seconds by timer1 overflow
    infra.interruptNotification();  // Notify the infrastructure of the interrupt
}

void loop() {
    oDriveContainer.tick();  // Tick the container
    infra.tick();            // Tick the infrastructure
}
