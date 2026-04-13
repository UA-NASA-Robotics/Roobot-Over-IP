#include <Arduino.h>
#include <ODriveUART.h>  //TODO: Change includes to be ch32v compatible (using hardware uart)
#include <stdint.h>
#include "oDriveError.h"
#if ODRIVE_MODULE_REV < 3
    #include <SoftwareSerial.h>
#elif ODRIVE_MODULE_REV == 3
    #include <HardwareTimer.h>
#endif

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

// TODO: Add Rev3 using ch32v common ini for configurations

#else
#error "ODrive module revision not supported, please set ODRV_MODULE_REV to 1 or 2"
#endif

#if ODRIVE_MODULE_REV <= 2 && ODRIVE_MODULE_REV >= 1  // Revision commonality section
    #define ODRV_RX 8
    #define ODRV_TX 7
    #define W5500_CS_PIN 10
#endif

#if ODRIVE_MODULE_REV == 3
    #define W5500_CS_PIN 10     

    #define ODRV_RX1 8
    #define ODRV_TX1 7

    #define ODRV_RX2 8
    #define ODRV_TX2 7

    #define ODRV_RX3 8
    #define ODRV_TX3 7

    #define ODRV_RX4 8
    #define ODRV_TX4 7
    
    #define ODRV_RX5 8
    #define ODRV_TX5 7
#endif

#if ODRIVE_MODULE_REV != 1 && ODRIVE_MODULE_REV != 2 && ODRIVE_MODULE_REV != 3
#error "ODrive module revision not supported, please set ODRV_MODULE_REV to 1, 2, or 3"
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

uint8_t* generalBuffer(nullptr);  // Sharing a large buffer from the infrastructure in this main.cpp
ModuleInfrastructure* infraRef(
    nullptr);  // Reference to the infrastructure for withing handleGeneralPacket function

#if ODRIVE_MODULE_REV == 3
ODriveController controller1(
    ODRV_RX1, ODRV_TX1, 115200,
    infraRef->moduleStatusManager);  // Create an instance of the ODriveController

ODriveController controller2(
    ODRV_RX2, ODRV_TX2, 115200,
    infraRef->moduleStatusManager);

ODriveController controller3(
    ODRV_RX3, ODRV_TX3, 115200,
    infraRef->moduleStatusManager); 

ODriveController controller4(
    ODRV_RX4, ODRV_TX4, 115200,
    infraRef->moduleStatusManager); 

ODriveController controller5(
    ODRV_RX5, ODRV_TX5, 115200,
    infraRef->moduleStatusManager);
#else
ODriveController controller1(
    ODRV_RX, ODRV_TX, 115200,
    infraRef->moduleStatusManager);  // Create an instance of the ODriveController
#endif

#if ODRIVE_MODULE_REV == 3
ODriveContainer<5> oDriveContainer;  // Create an instance of the ODriveContainer with 5 possible ODriveControllers
#else
ODriveContainer<1> oDriveContainer;  // Create an instance of the ODriveContainer
#endif

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

#ifdef CH32V
    oDriveContainer.append(controller1);  // Append the controller to the container
    oDriveContainer.append(controller2);  // Append the controller to the container
    oDriveContainer.append(controller3);  // Append the controller to the container
    oDriveContainer.append(controller4);  // Append the controller to the container
    oDriveContainer.append(controller5);  // Append the controller to the container
#else
    oDriveContainer.append(controller1);  // Append the controller to the container
#endif

    oDriveContainer.init();  // Initialize the container

#if USE_ROI_WATCHDOG
    infra.moduleStatusManager.setDisconnectCallback(staticPauseCallback);  // Set the pause
    callback infra.moduleStatusManager.setReconnectCallback(
        staticResumeCallback);  // Set the resume callback
#endif

    infra.moduleStatusManager.notifyInitializedStatus();  // Notify the infrastructure that the
                                                          // module has been initialized.

    // Hardware interrupt for CH32v
    #ifndef __AVR__
    // Initialize hardware timer interrupt for CH32v
    HardwareTimer timerInter(TIM6);
    
    // FIXME: Create dedicated interrupt callback functions, I don't know where to put it
    timerInter.attachInterrupt(std::bind(&ModuleInfrastructure::interruptNotification, &infra)); // Attach the infrastructure interrupt notification to the timer interrupt
    timerInter.setPrescaleFactor(8000); // at 8mHz, this gives 1kHz (I don't know if it's at 8mHz)
    timerInter.setOverflow(1000); // With 1kHz, this gives a 1 second overflow time
    timerInter.resume(); // Start the timer
    // getTimerClkFreq(), for checking hz when this program might eventually compile
    #endif
}

// Interrupt for AVR
#if defined(__AVR__)
ISR(TIMER1_OVF_vect) {
    // This ISR is called every 1.048 seconds by timer1 overflow

    infra.interruptNotification();  // Notify the infrastructure of the interrupt
}
#else
#endif



void loop() {
    oDriveContainer.tick();  // Tick the container
    infra.tick();            // Tick the infrastructure
}
