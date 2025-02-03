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

// Limit switches
LimitSwitch lower0(A2);
LimitSwitch upper0(A3);
LimitSwitch lower1(A4);
LimitSwitch upper1(A5);

// Encoders
FirgelliEncoder enc0(5, 4, 2, 3);
FirgelliEncoder enc1(5, 4, A1, A0);

// Motors
IBT2BinaryMotor motor0(9, 8);
IBT2BinaryMotor motor1(7, 6);

// Actuators
Actuator act0(&enc0, &motor0, &lower0, &upper0);
Actuator act1(&enc1, &motor1, &lower1, &upper1);

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