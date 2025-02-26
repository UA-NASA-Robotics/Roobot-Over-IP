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
#include "../include/EncoderDrivers/FirgelliEncoder.h"
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
Actuator act0(&enc0, &motor0, &upper0, &lower0, 0, 150);
Actuator act1(&enc1, &motor1, &upper1, &lower1, 0, 150);

uint8_t* generalBuffer(
    nullptr);  // Memory access for the general buffer [ROIConstants::ROI_MAX_PACKET_PAYLOAD] in len
ModuleInfrastructure* infraRef(nullptr);  // Memory access for the infrastructure

ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet packet) {
    return actuators.handleGeneralPacket(packet);
}

ModuleInfrastructure infra(10, 0, moduleTypesConstants::Actuator,
                           handleGeneralPacket);  // Create an instance of the infrastructure

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

    actuators.tick();  // Handle updates to the actuator's motor control
}