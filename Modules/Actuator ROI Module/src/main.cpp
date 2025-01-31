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

#include "../include/ActuatorContainer.h"
#include "../include/Encoders/FirgelliEncoder.h"
#include "../include/MotorDrivers/IBT2BinaryMotor.h"

// Actuator container
ActuatorContainer<2> actuators;

// Encoders
FirgelliEncoder enc0(0xFF, 0xFF, 0xFF, 0xFF);
FirgelliEncoder enc1(0xFF, 0xFF, 0xFF, 0xFF);

// Motors
IBT2BinaryMotor motor0(0xFF, 0xFF);
IBT2BinaryMotor motor1(0xFF, 0xFF);

// Actuators
Actuator act0(&enc0, &motor0);
Actuator act1(&enc1, &motor1);

uint8_t* generalBuffer(
    nullptr);  // Memory access for the general buffer [ROIConstants::ROIMAXPACKETPAYLOAD] in len
ModuleInfrastructure* infraRef(nullptr);  // Memory access for the infrastructure

// Function to handle a general packet
//@param packet The packet to handle
ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet packet) {
    uint16_t action = packet.getActionCode();  // Get the action code from the packet
    // uint16_t subDeviceID = packet.getSubDeviceID();  // Get the subdevice ID from the packet
    packet.getData(generalBuffer,
                   ROIConstants::ROIMAXPACKETPAYLOAD);  // Get the payload from the packet

    ROIPackets::Packet replyPacket = packet.swapReply();  // Create a reply packet

    return replyPacket;  // Return the reply packet
}

ModuleInfrastructure infra(10, 2, moduleTypesConstants::Actuator,
                           handleGeneralPacket);  // Create an infrastructure instance

void setup() {
    generalBuffer = &infra.generalBuffer[0];  // Get the general buffer from the infrastructure
    infraRef = &infra;                        // Get the infrastructure reference

    infra.init();  // Initialize the infrastructure
    
    // Connect the actuators to the container and initialize
    actuators.append(act0);
    actuators.append(act1);
    actuators.init();

    infra.moduleStatusManager.notifyInitializedStatus();  // Notify the status manager that the
                                                          // module has been initialized
}

ISR(TIMER1_OVF_vect) {
    // This ISR is called every 1.048 seconds by timer1 overflow
    infra.interruptNotification();  // Notify the infrastructure of the interrupt
}

void loop() {
    infra.tick();  // Process packets in the loop

    actuators.tick();   // Handle updates to the actuator's motor control
}