#include "infrastructure.h"

ModuleInfrastructure::ModuleInfrastructure(uint8_t W5500_CS_Pin, uint8_t octetSelectorREV,
                                           moduleTypesConstants::moduleTypeConstant moduleType,
                                           ROIPackets::Packet (*handler)(ROIPackets::Packet))
    : WIZ5500_CS_PIN(W5500_CS_Pin),
      selector(nullptr),
      moduleIPContainer(InfrastructureConstants::NETWORK_ADDRESS1,
                        InfrastructureConstants::NETWORK_ADDRESS2,
                        InfrastructureConstants::NETWORK_ADDRESS3, 5),
      moduleChainManager(moduleType, moduleIPContainer, moduleStatusManager, SysAdmin,
                         generalBuffer),
      moduleSysAdminHandler(moduleType, moduleStatusManager, moduleChainManager,
                            moduleBlacklistManager, generalBuffer),
      handleGeneralPacket(handler) {
    this->WIZ5500_CS_PIN = W5500_CS_Pin;
    this->moduleType = moduleType;

    switch (octetSelectorREV) {
        case 0:
            selector = new OctetSelectorRevNull();
            break;
        case 1:
            selector = new OctetSelectorRev1();
            break;
        case 2:
            selector = new OctetSelectorRev2();
            break;
        default:
            selector = new OctetSelectorRev1();
            break;
    }
}

void ModuleInfrastructure::interruptNotification() { moduleChainManager.notifyDoDiscovery(); }

#if defined(__AVR__)
void ModuleInfrastructure::setHardwareInterruptTimer() {
    // ISR for Chain Discovery setup
    TCCR1A = 0;  // set entire TIMER1 to zero, and initialize the timer1 registers
    TCCR1B = 0;

    TCCR1B |= 0b00000100;  // set the timer1 prescaler to 256. IE 16MHz/256 = 62500Hz. Timer1
                           // overflows at 65535, so 65535/62500 = 1.048 seconds

    TIMSK1 |= 0b00000001;  // enable timer1 overflow interrupt
    // End of ISR setup
}
#endif

void ModuleInfrastructure::init() {
#if defined(__AVR__) && DEBUG
    Serial.begin(115200);  // Initialize the serial port for debugging
#endif

#if defined(__AVR__)
    delay(100);  // Wait for devices to initialize
#else
// non AVR
#endif

    if (handleGeneralPacket == nullptr) {
#if defined(__AVR__) && DEBUG
        Serial.println(F("No handler function provided, cannot initialize infrastructure"));
#endif
        while (true) {
            ;
        }
    }

#if defined(__AVR__)
    setHardwareInterruptTimer();
#endif

    selector->init();  // Initialize the octet selector

    moduleIPContainer.updateIP(selector->readOctet());  // Initialize the IP container and read the
    // selector

    macHelper.getMac(
        mac);  // Get the MAC address from the EEPROM, or generate one if it doesn't exist
    moduleSysAdminHandler.setMAC(mac);  // Set the MAC address in the sysAdminHandler

    Ethernet.init(WIZ5500_CS_PIN);  // Initialize the Ethernet module SPI interface
    Ethernet.begin(mac, moduleIPContainer.networkAddress);  // Initialize the Ethernet module
                                                            // with the MAC and IP addresses

    w5500.setRetransmissionCount(
        InfrastructureConstants::RETRANSMISSION_COUNT);  // Set the retransmission count to 1,
                                                         // ie 2 attempts
    w5500.setRetransmissionTime(
        InfrastructureConstants::RETRANSMISSION_TIME);  // Set the retransmission time to 10ms

    if (w5500.readPHYCFGR() && 0x01 == 0) {  // Check if the link status is connected
#if defined(__AVR__) && DEBUG
        Serial.println(F("Ethernet not connected."));
#endif
        while (w5500.readPHYCFGR() && 0x01 == 0) {
#if defined(__AVR__)
            delay(100);  // Wait for the Ethernet cable to be connected
#else
// non AVR
#endif
        }
#if defined(__AVR__) && DEBUG
        Serial.println(F("Ethernet connected. Resuming operation."));
#endif
    }

    General.begin(ROIConstants::ROI_GENERAL_PORT);  // Initialize the general UDP instance
    // Interrupt.begin(ROIConstants::ROI_STREAM_PORT);  // Initialize the interrupt UDP instance
    SysAdmin.begin(ROIConstants::ROI_SYS_ADMIN_PORT);  // Initialize the sysAdmin UDP instance

#if defined(__AVR__)
    delay(500);  // Wait for devices to initialize
#else
// non AVR
#endif

#if defined(__AVR__) && DEBUG
    Serial.println(F("Module is ready for operation."));
#endif
}

void ModuleInfrastructure::tick() {
    // Check for connection status
    if (w5500.readPHYCFGR() && 0x01 == 0) {
#if defined(__AVR__) && DEBUG
        Serial.println(F("Ethernet cable is not connected. Reinitalizing."));
        delay(1000);  // delay for 1 second for serial to print
#endif
        resetFunction();  // The reset function is called to restart the module
        // The program will not fully resume operation until the Ethernet cable is
        // connected
    }

    // Check for a general packet
    int generalPacketSize = General.parsePacket();
    if (generalPacketSize) {
        IPAddress remote = General.remoteIP();           // Get the remote IP address
        General.read(generalBuffer, generalPacketSize);  // Read the general packet

        if (!InfrastructureConstants::IGNORE_BLACKLIST &&
            moduleBlacklistManager.verifyOctet(
                remote[3])) {  // Check if the remote IP is blacklisted
            return;            // stop parsing the packet if the remote IP is blacklisted
        }

        moduleStatusManager.notifyPacketReceived();  // Notify the status manager that a packet was
                                                     // received

        ROIPackets::Packet generalPacket(moduleIPContainer.networkAddress[3],
                                         remote[3]);  // Create a general packet from the buffer

        if (!generalPacket.importPacket(generalBuffer,
                                        ROIConstants::ROI_MAX_PACKET_SIZE) &&
            !InfrastructureConstants::IGNORE_CHECKSUM_FAILURE) {  // Import the general packet from
                                                                  // the buffer
#if defined(__AVR__) && DEBUG
            Serial.println(F("Failed to import general packet"));
#endif
            return;  // Skip the rest of the loop if the packet failed to import
        }

        ROIPackets::Packet replyPacket =
            handleGeneralPacket(generalPacket);  // Handle the general packet

        replyPacket.exportPacket(
            generalBuffer,
            ROIConstants::ROI_MAX_PACKET_SIZE);  // Export the reply packet to the buffer
        if (!General.beginPacket(remote,
                                 ROIConstants::ROI_GENERAL_PORT)) {  // Begin the reply packet
#if defined(__AVR__) && DEBUG
            Serial.println(F("Failed to begin general packet"));
            Serial.print(F("To remote host: "));
            Serial.println(remote[3]);
#endif
        }

        General.write(generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);
        if (!General.endPacket()) {  // Send the reply packet
#if defined(__AVR__) && DEBUG
            Serial.println(F("Failed to send general packet"));
            Serial.print(F("To remote host: "));
            Serial.println(remote[3]);
#endif
        }
    }

    // Check for an interrupt packet
    // todo

    // Check for a sysAdmin packet
    int sysAdminPacketSize = SysAdmin.parsePacket();
    if (sysAdminPacketSize) {
        IPAddress remote = SysAdmin.remoteIP();            // Get the remote IP address
        SysAdmin.read(generalBuffer, sysAdminPacketSize);  // Read the general packet

        if (!InfrastructureConstants::IGNORE_BLACKLIST &&
            moduleBlacklistManager.verifyOctet(
                remote[3])) {  // Check if the remote IP is blacklisted
            return;            // stop parsing the packet if the remote IP is blacklisted
        }

        ROIPackets::sysAdminPacket sysAdminPacket(
            moduleIPContainer.networkAddress[3],
            remote[3]);  // Create a general packet from the buffer

        if (!sysAdminPacket.importPacket(generalBuffer,
                                         ROIConstants::ROI_MAX_PACKET_SIZE) &&
            !InfrastructureConstants::IGNORE_CHECKSUM_FAILURE) {  // Import the sysadmin packet from
                                                                  // the
                                                                  // buffer

#if defined(__AVR__) && DEBUG
            Serial.println(F("Failed to import sysadmin packet"));
#endif

            return;  // Skip the rest of the loop if the packet failed to import
        }

        ROIPackets::sysAdminPacket replyPacket = moduleSysAdminHandler.handleSysAdminPacket(
            sysAdminPacket);  // Handle the sysAdmin packet

        if (replyPacket.getActionCode() == sysAdminConstants::BLANK) {
#if defined(__AVR__) && DEBUG
            Serial.println(F("Blank packet received, no reply needed"));
#endif
            return;
        }

        replyPacket.exportPacket(
            generalBuffer,
            ROIConstants::ROI_MAX_PACKET_SIZE);  // Export the reply packet to the buffer

        if (!SysAdmin.beginPacket(remote,
                                  ROIConstants::ROI_SYS_ADMIN_PORT)) {  // Begin the reply packet
#if defined(__AVR__) && DEBUG
            Serial.println(F("Failed to begin sysadmin packet"));
            Serial.print(F("To remote host: "));
            Serial.println(remote[3]);
#endif
        }
        SysAdmin.write(generalBuffer, ROIConstants::ROI_MAX_PACKET_SIZE);
        if (!SysAdmin.endPacket()) {
#if defined(__AVR__) && DEBUG
            Serial.println(F("Failed to send sysadmin packet"));
            Serial.print(F("To remote host: "));
            Serial.println(remote[3]);
#endif
        }
    }

    moduleChainManager.discoverChain();  // Discover the chain neighbors (Does nothing
                                         // if not activated by ISR)

    moduleStatusManager.tickDisconnectWatchdog();  // Tick the disconnect watchdog to check for
                                                   // connection timeouts
}
