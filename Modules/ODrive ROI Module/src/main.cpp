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
#include "../../../lib/moduleLib/blacklistManager.h"
#include "../../../lib/moduleLib/chainNeighborManager.h"
#include "../../../lib/moduleLib/macGen.h"
#include "../../../lib/moduleLib/statusManager.h"
#include "../../../lib/moduleLib/sysAdminHandler.h"

using namespace ODriveConstants;  // Import the constants from the ODriveConstants namespace
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

BlacklistManager moduleBlacklistManager;  // Create a blacklist manager instance

// Create a UDP instances for each type of packet on the ROI module
EthernetUDP General;
EthernetUDP Interrupt;
EthernetUDP SysAdmin;

uint8_t generalBuffer[ROIConstants::ROIMAXPACKETSIZE];  // Buffer for packet import and export

chainNeighborManager::chainNeighborManager moduleChainManager(
    moduleTypesConstants::ODrive, IPArray, IPArray[3], moduleStatusManager, SysAdmin,
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

    macHelper.getMac(
        mac);  // Get the MAC address from the EEPROM, or generate one if it doesn't exist
    moduleSysAdminHandler.setMAC(mac);  // Set the MAC address in the sysAdminHandler

    Ethernet.init(WIZ5500_CS_PIN);  // Initialize the Ethernet module SPI interface
    Ethernet.begin(mac, IP);        // Initialize the Ethernet module with the MAC and IP addresses

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

    General.begin(ROIConstants::ROIGENERALPORT);     // Initialize the general UDP instance
    Interrupt.begin(ROIConstants::ROIINTERUPTPORT);  // Initialize the interrupt UDP instance
    SysAdmin.begin(ROIConstants::ROISYSADMINPORT);   // Initialize the sysAdmin UDP instance

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
    while (odrive.getState() !=
           AXIS_STATE_CLOSED_LOOP_CONTROL) {  // set the ODrive to closed loop control
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
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

    switch (action) {
            ///-----------------SETTERS-----------------///
        case ODriveConstants::SETCONTROLMODE:
            oDriveControlMode = generalBuffer[0];  // Set the control mode of the ODrive
            setControlInputMode(oDriveControlMode, oDriveInputMode);

#if DEBUG
            Serial.println("Control Mode Set");
            Serial.println(generalBuffer[0]);
#endif

            replyPacket.setData(1);  // return 1 for success
            break;

        case ODriveConstants::SETINPUTMODE:
            setControlInputMode(oDriveControlMode, generalBuffer[0]);

            replyPacket.setData(1);  // return 1 for success
            break;

        case ODriveConstants::SETTORQUE:
            desiredTorque =
                floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
            applyFeeds();                                 // Apply the feeds to the ODrive

            replyPacket.setData(1);  // return 1 for success
            break;

        case ODriveConstants::SETPOSITION:
            desiredPosition =
                floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
#if DEBUG
            Serial.println(desiredPosition);
#endif
            applyFeeds();  // Apply the feeds to the ODrive

            replyPacket.setData(1);  // return 1 for success
            break;

        case ODriveConstants::SETRELATIVEPOSITION:
            desiredPosition +=
                floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
#if DEBUG
            Serial.println(desiredPosition);
            Serial.println(generalBuffer[0]);
            Serial.println(generalBuffer[1]);
            Serial.println(generalBuffer[2]);
            Serial.println(generalBuffer[3]);
#endif
            applyFeeds();  // Apply the feeds to the ODrive

            replyPacket.setData(1);  // return 1 for success
            break;

        case ODriveConstants::SETVELOCITY:
            desiredVelocity =
                floatCast::toFloat(generalBuffer, 0, 3);  // Convert the bytes to a float
            applyFeeds();                                 // Apply the feeds to the ODrive

#if DEBUG
            Serial.println(desiredVelocity);
#endif

            replyPacket.setData(1);  // return 1 for success
            break;

            ///-----------------GETTERS-----------------///

        case ODriveConstants::GETCONTROLMDODE:
            replyPacket.setData(oDriveControlMode);
            break;

        case ODriveConstants::GETINPUTMODE:
            replyPacket.setData(oDriveInputMode);
            break;

        case ODriveConstants::GETPOSITIONSETPOINT: {
            uint8_t* posBytes =
                reinterpret_cast<uint8_t*>(&desiredPosition);  // Convert the float to bytes

            replyPacket.setData(posBytes[0], posBytes[1], posBytes[2], posBytes[3]);
            break;
        }

        case ODriveConstants::GETVELOCITYSETPOINT: {
            uint8_t* velBytes =
                reinterpret_cast<uint8_t*>(&desiredVelocity);  // Convert the float to bytes

            replyPacket.setData(velBytes[0], velBytes[1], velBytes[2], velBytes[3]);
            break;
        }

        case ODriveConstants::GETTORQUESETPOINT: {
            uint8_t* torqBytes =
                reinterpret_cast<uint8_t*>(&desiredTorque);  // Convert the float to bytes

            replyPacket.setData(torqBytes[0], torqBytes[1], torqBytes[2], torqBytes[3]);
            break;
        }

            /// -----------------Errors-----------------///
        case ODriveConstants::GETERROR: {
            uint32_t error = odrive.getParameterAsInt(F("axis0.active_errors"));

            replyPacket.setData((error >> 24) & 0xFF, (error >> 16) & 0xFF, (error >> 8) & 0xFF,
                                error & 0xFF);
            break;
        }

        case ODriveConstants::CLEARERRORS:
            odrive.clearErrors();
            moduleStatusManager.notifyClearError();  // Notify the status manager that the
                                                     // module has cleared errors
            replyPacket.setData(1);                  // return 1 for success
            break;

            // -----------------Get Real-----------------/// (Live Real Data from ODrive)
        case ODriveConstants::GETPOSITION: {
            float pos = odrive.getPosition();
            uint8_t* posBytes = reinterpret_cast<uint8_t*>(&pos);  // Convert the float to bytes

            replyPacket.setData(posBytes[0], posBytes[1], posBytes[2], posBytes[3]);
            break;
        }

        case ODriveConstants::GETVELOCITY: {
            float vel = odrive.getVelocity();
            uint8_t* velBytes = reinterpret_cast<uint8_t*>(&vel);  // Convert the float to bytes

            replyPacket.setData(velBytes[0], velBytes[1], velBytes[2], velBytes[3]);
            break;
        }

        case ODriveConstants::GETBUSVOLTAGE: {
            float busVoltage = odrive.getParameterAsFloat(F("vbus_voltage"));
            uint8_t* busVoltageBytes =
                reinterpret_cast<uint8_t*>(&busVoltage);  // Convert the float to bytes

            replyPacket.setData(busVoltageBytes[0], busVoltageBytes[1], busVoltageBytes[2],
                                busVoltageBytes[3]);
            break;
        }

        case ODriveConstants::GETCURRENT: {
            float current = odrive.getParameterAsFloat(F("ibus"));
            uint8_t* currentBytes =
                reinterpret_cast<uint8_t*>(&current);  // Convert the float to bytes

            replyPacket.setData(currentBytes[0], currentBytes[1], currentBytes[2], currentBytes[3]);
            break;
        }

        case ODriveConstants::GETFETTEMPERATURE: {
            float fetTemp = odrive.getParameterAsFloat(F("axis0.motor.fet_thermistor.temperature"));
            uint8_t* fetTempBytes =
                reinterpret_cast<uint8_t*>(&fetTemp);  // Convert the float to bytes

            replyPacket.setData(fetTempBytes[0], fetTempBytes[1], fetTempBytes[2], fetTempBytes[3]);
            break;
        }

        case ODriveConstants::GETMOTORTEMPERATURE: {
            float motorTemp =
                odrive.getParameterAsFloat(F("axis0.motor.motor_thermistor.temperature"));
            uint8_t* motorTempBytes =
                reinterpret_cast<uint8_t*>(&motorTemp);  // Convert the float to bytes

            replyPacket.setData(motorTempBytes[0], motorTempBytes[1], motorTempBytes[2],
                                motorTempBytes[3]);
        }

        default:
            break;
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

        ROIPackets::Packet generalPacket(IPArray[3],
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
            IPArray[3],
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
