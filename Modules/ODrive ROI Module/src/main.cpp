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

// Release Versioning

#ifndef ODRV_MODULE_REV  // Revision differentials section
#define ODRV_MODULE_REV 1
#endif

#if ODRV_MODULE_REV == 1
#define OCTET_SELECTOR_REV 1

#elif ODRV_MODULE_REV == 2
#define OCTET_SELECTOR_REV 2

#else
#error "ODrive module revision not supported, please set ODRV_MODULE_REV to 1 or 2"
#endif

#if ODRIVE_MODULE_REV <= 2 && ODRIVE_MODULE_REV >= 1  // Revision commonality section
#define ODRV_RX 8
#define ODRV_TX 7
#define W5500_CS_PIN 10
#else
#error "ODrive module revision not supported, please set ODRV_MODULE_REV to 1 or 2"
// Default to revision 1 if not defined
#define ODRV_RX 8
#define ODRV_TX 7
#define W5500_CS_PIN 10
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
    ODRV_RX, ODRV_TX, 115200,
    infraRef->moduleStatusManager);  // Create an instance of the ODriveController

ODriveContainer<1> oDriveContainer;  // Create an instance of the ODriveContainer

ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet packet) {
    return oDriveContainer.handleGeneralPacket(packet);
}

void staticPauseCallback() { oDriveContainer.pause(); }

void staticResumeCallback() { oDriveContainer.resume(); }

ModuleInfrastructure infra(W5500_CS_PIN, OCTET_SELECTOR_REV, moduleTypesConstants::O_DRIVE,
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
