#include <Arduino.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#include <ODriveUART.h>
#include <SoftwareSerial.h>
#include <stdint.h>

// Define the default debug mode for the ROI module
#ifndef DEBUG
#define DEBUG false
#endif
// Modify debug mode in "PlatformIO.ini" file, NOT here

#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/floatCast.h"
#include "../../../lib/moduleLib/IPContainer.h"
#include "../../../lib/moduleLib/blacklistManager.h"
#include "../../../lib/moduleLib/chainNeighborManager.h"
#include "../../../lib/moduleLib/macGen.h"
#include "../../../lib/moduleLib/octetSelector.h"
#include "../../../lib/moduleLib/statusManager.h"
#include "../../../lib/moduleLib/sysAdminHandler.h"

using namespace ODriveConstants;  // Import the constants from the ODriveConstants namespace
                                  // namespace as we will be using them in this file

// Hardware CONSTANTS
const uint8_t WIZ5500_CS_PIN = 10;  // Chip select pin for WIZ5500 module

macGen::macAddressHelper macHelper;
uint8_t mac[6];

OctetSelectorRev1 selector;  // Create an octet selector instance

IPContainer moduleIPContainer(&selector, (uint8_t)10, (uint8_t)0, (uint8_t)0);

statusManager::statusManager
    moduleStatusManager;  // Create a status manager instance (manages the status of the ROI module)

BlacklistManager moduleBlacklistManager;  // Create a blacklist manager instance

// Create a UDP instances for each type of packet on the ROI module
EthernetUDP General;
EthernetUDP Interrupt;
EthernetUDP SysAdmin;

uint8_t generalBuffer[ROIConstants::ROIMAXPACKETSIZE];  // Buffer for packet import and export

chainNeighborManager::chainNeighborManager moduleChainManager(
    moduleTypesConstants::ODrive, moduleIPContainer, moduleStatusManager, SysAdmin,
    generalBuffer);  // Create a chainNeighborManager instance

sysAdminHandler::sysAdminHandler moduleSysAdminHandler(
    moduleTypesConstants::ODrive, moduleStatusManager, moduleChainManager, moduleBlacklistManager,
    generalBuffer);  // Create a sysAdminHandler instance

// --- ODrive Stuff ---

SoftwareSerial odrive_serial(8, 7);  // RX, TX
unsigned long baudrate =
    19200;  // Baudrate of the ODrive, this is the max software serial can handle reliably

ODriveUART odrive(odrive_serial);  // Create an ODriveUART instance

uint8_t oDriveControlMode = ODriveConstants::POSITIONMODE;  // Default control mode is position mode
uint8_t oDriveInputMode = ODriveConstants::AUTO_BEST_FIT_MODE;  // Default input mode is auto best
                                                                // fit mode

float desiredPosition = 0;  // Desired position of the ODrive
float desiredVelocity = 0;  // Desired velocity of the ODrive
float desiredTorque = 0;    // Desired torque of the ODrive

void setup() {
    // ISR for Chain Discovery setup
    TCCR1A = 0;  // set entire TIMER1 to zero, and initialize the timer1 registers
    TCCR1B = 0;

    TCCR1B |= 0b00000100;  // set the timer1 prescaler to 256. IE 16MHz/256 = 62500Hz. Timer1
                           // overflows at 65535, so 65535/62500 = 1.048 seconds

    TIMSK1 |= 0b00000001;  // enable timer1 overflow interrupt
    // End of ISR setup

    selector.init();  // Initialize the octet selector

    moduleIPContainer.init();  // Initialize the IP container and read the selector

    macHelper.getMac(
        mac);  // Get the MAC address from the EEPROM, or generate one if it doesn't exist
    moduleSysAdminHandler.setMAC(mac);  // Set the MAC address in the sysAdminHandler

    Ethernet.init(WIZ5500_CS_PIN);  // Initialize the Ethernet module SPI interface
    Ethernet.begin(
        mac, moduleIPContainer
                 .networkAddress);  // Initialize the Ethernet module with the MAC and IP addresses

    w5500.setRetransmissionCount(1);  // Set the retransmission count to 1, ie 2 attempts
    w5500.setRetransmissionTime(10);  // Set the retransmission time to 10ms

    odrive_serial.begin(baudrate);  // Initialize the software serial port for the ODrive

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

    General.begin(ROIConstants::ROIGENERALPORT);  // Initialize the general UDP instance
    // Interrupt.begin(ROIConstants::ROIINTERUPTPORT);  // Initialize the interrupt UDP instance
    SysAdmin.begin(ROIConstants::ROISYSADMINPORT);  // Initialize the sysAdmin UDP instance

    delay(500);  // Wait for devices to initialize

    moduleStatusManager.notifyInitializedStatus();  // Notify the status manager that the module is
    // initialized and ready for operation

#if DEBUG
    Serial.println("Waiting for ODrive...");
#endif
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
        delay(100);
    }

#if DEBUG
    Serial.println("Enabling closed loop control...");
#endif

    for (int i = 0; i < 10; i++) {  // try to enable closed loop control 10 times
        if (odrive.getState() == AXIS_STATE_CLOSED_LOOP_CONTROL) {
            break;
        }
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
    }

    if (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        // if we couldn't enable closed loop control, lockout as critical error is unresolvable
        while (1) {
            Serial.println("Critical Error: Unable to enable closed loop control. Halted.");
            delay(1000);
        }
    }

#if DEBUG
    Serial.println(F("ODrive ROI Module is ready for operation."));
#endif
}

ISR(TIMER1_OVF_vect) {
    // This ISR is called every 1.048 seconds by timer1 overflow
    moduleChainManager.notifyDoDiscovery();  // Notify the chain manager to do discovery on the next
                                             // void loop cycle
}

void (*resetFunction)(void) = 0;  // declare reset function @ address 0

/**
 * @brief Set the Control and Input mode of the ODrive
 *
 * @param controlMode , the control mode to set the ODrive to (ROI VALUE)
 * @param inputMode , the input mode to set the ODrive to (ROI VALUE)
 * @return true, if the control and input mode were set successfully
 * @return false, if the control and input mode were not set successfully
 */
bool setControlInputMode(uint8_t controlMode, uint8_t inputMode) {
    // Set the ODrive input mode based on the parameter
    uint8_t ODriveInputModeVal = 0;
    switch (inputMode) {
        case ODriveConstants::TRAP_TRAJ_MODE:
            ODriveInputModeVal = INPUT_MODE_TRAP_TRAJ;
            break;
        case ODriveConstants::POS_FILTER_MODE:
            ODriveInputModeVal = INPUT_MODE_POS_FILTER;
            break;
        case ODriveConstants::VELOCITY_RAMP_MODE:
            ODriveInputModeVal = INPUT_MODE_VEL_RAMP;
            break;
        case ODriveConstants::TORQUE_RAMP_MODE:
            ODriveInputModeVal = INPUT_MODE_TORQUE_RAMP;
            break;
        case ODriveConstants::AUTO_BEST_FIT_MODE:
            switch (controlMode) {
                case ODriveConstants::POSITIONMODE:
                    ODriveInputModeVal = INPUT_MODE_TRAP_TRAJ;
                    break;
                case ODriveConstants::VELOCITYMODE:
                    ODriveInputModeVal = INPUT_MODE_VEL_RAMP;
                    break;
                case ODriveConstants::TORQUEMODE:
                    ODriveInputModeVal = INPUT_MODE_TORQUE_RAMP;
                    break;
                default:
                    break;
            };
    };

    // Set the ODrive control mode based on the parameter
    uint8_t ODriveControlModeVal = 0;
    switch (controlMode) {
        case ODriveConstants::POSITIONMODE:
            ODriveControlModeVal = CONTROL_MODE_POSITION_CONTROL;
            break;
        case ODriveConstants::VELOCITYMODE:
            ODriveControlModeVal = CONTROL_MODE_VELOCITY_CONTROL;
            break;
        case ODriveConstants::TORQUEMODE:
            ODriveControlModeVal = CONTROL_MODE_TORQUE_CONTROL;
            break;
        default:
            break;
    };

    // issue the commands to the ODrive
    odrive.setParameter(F("axis0.controller.config.control_mode"), controlMode);
    odrive.setParameter(F("axis0.controller.config.input_mode"), inputMode);

    return true;  // no error handling yet
}

/**
 * @brief Sends the desired position/velocity/torque to the ODrive. It applies feed forwards when
 * applicable
 *
 * @return true, if the desired position/velocity/torque was sent successfully
 * @return false, if the desired position/velocity/torque was not sent successfully
 */
bool applyFeeds() {
    switch (oDriveControlMode) {
        case ODriveConstants::POSITIONMODE:
            odrive.setPosition(desiredPosition, desiredVelocity, desiredTorque);
            break;

        case ODriveConstants::VELOCITYMODE:
            odrive.setVelocity(desiredVelocity, desiredTorque);
            break;

        case ODriveConstants::TORQUEMODE:
            odrive.setTorque(desiredTorque);
            break;

        default:
            return false;
            break;
    }

    return true;
}

// Function to handle a general packet
//@param packet The packet to handle
ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet packet) {
    uint16_t action = packet.getActionCode();  // Get the action code from the packet
    // uint16_t subDeviceID = packet.getSubDeviceID();  // Get the subdevice ID from the packet
    packet.getData(generalBuffer,
                   ROIConstants::ROIMAXPACKETPAYLOAD);  // Get the payload from the packet

    ROIPackets::Packet replyPacket = packet.swapReply();  // Create a reply packet

    if (action & MaskConstants::SETMASK) {             // Split the code into setters and getters
        switch (action & (!MaskConstants::SETMASK)) {  // remove the set mask

            case ODriveConstants::MaskConstants::ControlMode:
                oDriveControlMode = generalBuffer[0];  // Set the control mode of the ODrive
                setControlInputMode(oDriveControlMode, oDriveInputMode);
#if DEBUG
                Serial.print("Control Mode Set:");
                Serial.println(generalBuffer[0]);
#endif
                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::InputMode:
                oDriveInputMode = generalBuffer[0];  // Set the input mode of the ODrive
                setControlInputMode(oDriveControlMode, oDriveInputMode);

#if DEBUG
                Serial.print("Input Mode Set:");
                Serial.println(generalBuffer[0]);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::Torque:
                desiredTorque =
                    floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
                applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
                Serial.print("Torque Set:");
                Serial.println(desiredTorque);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::PositionSetPoint:
                desiredPosition =
                    floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
                applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
                Serial.print("Position Set:");
                Serial.println(desiredPosition);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::VelocitySetPoint:
                desiredVelocity =
                    floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
                applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
                Serial.print("Velocity Set:");
                Serial.println(desiredVelocity);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::PositionRelative:
                desiredPosition +=
                    floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
                applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
                Serial.print("Relative Position Set:");
                Serial.println(desiredPosition);
#endif

                replyPacket.setData(1);  // return 1 for success
                break;

            case ODriveConstants::MaskConstants::Error:
                odrive.clearErrors();
                moduleStatusManager.notifyClearError();  // Notify the status manager that the
                                                         // module has cleared errors

#if DEBUG
                Serial.println("Errors Cleared");
#endif

                replyPacket.setData(1);  // return 1 for success

                break;

            default:
                Serial.print("Unknown Action: ");
                Serial.println(action);
                break;
        }
    } else {
        switch (action & (!ODriveConstants::MaskConstants::SETMASK)) {
            case ODriveConstants::MaskConstants::ControlMode:
                replyPacket.setData(oDriveControlMode);
                break;

            case ODriveConstants::MaskConstants::InputMode:
                replyPacket.setData(oDriveInputMode);
                break;

            case ODriveConstants::MaskConstants::Torque: {
                uint8_t torqueBytes[4];
                floatCast::floatToUint8Array(desiredTorque, torqueBytes, 0, 3);

                replyPacket.setData(torqueBytes, 4);  // Set the data in the reply packet
            }

            case ODriveConstants::MaskConstants::PositionSetPoint: {
                uint8_t posBytes[4];
                floatCast::floatToUint8Array(desiredPosition, posBytes, 0, 3);

                replyPacket.setData(posBytes, 4);  // Set the data in the reply packet

                break;
            }

            case ODriveConstants::MaskConstants::VelocitySetPoint: {
                uint8_t velBytes[4];
                floatCast::floatToUint8Array(desiredVelocity, velBytes, 0, 3);

                replyPacket.setData(velBytes, 4);  // Set the data in the reply packet

                break;

                case ODriveConstants::MaskConstants::Position: {
                    float pos = odrive.getPosition();
                    uint8_t posBytes[4];
                    floatCast::floatToUint8Array(pos, posBytes, 0, 3);

                    replyPacket.setData(posBytes, 4);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::Velocity: {
                    float vel = odrive.getVelocity();
                    uint8_t velBytes[4];
                    floatCast::floatToUint8Array(vel, velBytes, 0, 3);

                    replyPacket.setData(velBytes, 4);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::BusVoltage: {
                    float busVoltage = odrive.getParameterAsFloat(F("vbus_voltage"));
                    uint8_t busVoltageBytes[4];
                    floatCast::floatToUint8Array(busVoltage, busVoltageBytes, 0, 3);

                    replyPacket.setData(busVoltageBytes, 4);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::Current: {
                    float current = odrive.getParameterAsFloat(F("ibus"));
                    uint8_t currentBytes[4];
                    floatCast::floatToUint8Array(current, currentBytes, 0, 3);

                    replyPacket.setData(currentBytes, 4);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::FETTemperature: {
                    float fetTemp =
                        odrive.getParameterAsFloat(F("axis0.motor.fet_thermistor.temperature"));
                    uint8_t fetTempBytes[4];
                    floatCast::floatToUint8Array(fetTemp, fetTempBytes, 0, 3);

                    replyPacket.setData(fetTempBytes, 4);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::MotorTemperature: {
                    float motorTemp =
                        odrive.getParameterAsFloat(F("axis0.motor.motor_thermistor.temperature"));
                    uint8_t motorTempBytes[4];
                    floatCast::floatToUint8Array(motorTemp, motorTempBytes, 0, 3);

                    replyPacket.setData(motorTempBytes, 4);  // Set the data in the reply packet

                    break;
                }

                case ODriveConstants::MaskConstants::Error: {
                    uint32_t error = odrive.getParameterAsInt(F("axis0.active_errors"));

                    replyPacket.setData((error >> 24) & 0xFF, (error >> 16) & 0xFF,
                                        (error >> 8) & 0xFF,
                                        error & 0xFF);  // Set the data in the reply packet

                    break;
                }
            }
        }
    }
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
        // The program will not fully resume operation until the Ethernet cable is
        // connected
    }
    // Check for ODrive connection
    if (odrive.getState() == AXIS_STATE_UNDEFINED) {
#if DEBUG
        Serial.println(F("ODrive is not connected. Reinitalizing."));
        delay(1000);  // delay for 1 second for serial to print
#endif
        resetFunction();  // The reset function is called to restart the module
        // The program will not fully resume operation until the ODrive is connected
    }

    if (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        moduleStatusManager.notifySystemError(true);  // set the system as inoperable. Needs reset.
#if DEBUG
        Serial.println(
            F("ODrive is not in closed loop control. System is inoperable. Likely ODrive "
              "error."));
#endif
    }

    // Check for a general packet
    int generalPacketSize = General.parsePacket();
    if (generalPacketSize) {
        IPAddress remote = General.remoteIP();           // Get the remote IP address
        General.read(generalBuffer, generalPacketSize);  // Read the general packet

        if (moduleBlacklistManager.verifyOctet(
                remote[3])) {  // Check if the remote IP is blacklisted
            return;            // stop parsing the packet if the remote IP is blacklisted
        }

        ROIPackets::Packet generalPacket(moduleIPContainer.networkAddress[3],
                                         remote[3]);  // Create a general packet from the buffer
        generalPacket.importPacket(generalBuffer,
                                   ROIConstants::ROIMAXPACKETSIZE);  // Import the general
                                                                     // packet from the buffer

        ROIPackets::Packet replyPacket =
            handleGeneralPacket(generalPacket);  // Handle the general packet

        replyPacket.exportPacket(
            generalBuffer,
            ROIConstants::ROIMAXPACKETSIZE);  // Export the reply packet to the buffer
        General.beginPacket(remote,
                            ROIConstants::ROIGENERALPORT);  // Begin the reply packet
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

        if (moduleBlacklistManager.verifyOctet(
                remote[3])) {  // Check if the remote IP is blacklisted
            return;            // stop parsing the packet if the remote IP is blacklisted
        }

        ROIPackets::sysAdminPacket sysAdminPacket(
            moduleIPContainer.networkAddress[3],
            remote[3]);  // Create a general packet from the buffer
        bool success =
            sysAdminPacket.importPacket(generalBuffer,
                                        ROIConstants::ROIMAXPACKETSIZE);  // Import the general
                                                                          // packet from the buffer
        if (!success) {
#if DEBUG
            Serial.println(F("Failed to import sysadmin packet"));
#endif

            //-------------------------------return;  // Skip the rest of the loop if
            // the packet
            // failed to import
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
            SysAdmin.beginPacket(remote,
                                 ROIConstants::ROISYSADMINPORT);  // Begin the reply packet
            SysAdmin.write(generalBuffer, ROIConstants::ROIMAXPACKETSIZE);
            if (SysAdmin.endPacket()) {  // if the packet was sent successfully quit the
                                         // loop
                sent = 10;
            }  // Send the reply packet
            sent++;
        }
    }

    moduleChainManager.discoverChain();  // Discover the chain neighbors (Does nothing
                                         // if not activated by ISR)
}
