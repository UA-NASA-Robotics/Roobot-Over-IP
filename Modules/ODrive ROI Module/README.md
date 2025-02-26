# ODrive Module

# Firmware

The ODrive module extends off of the typical ROI Infrastructure system.
While the atmega328pb only supports 1 software UART, and therefore only 1 ODrive can be connected, the firmware is built in OOP to support an arbitrary number of ODrives.

Each ODrive gets a virtual ODriveController object, which is responsible for handling the communication with the ODrive and provides essentially all ROI interaction functionality.

There is an ODriveContainer templatized class that holds the ODriveController objects, and passes the packets to the appropriate ODriveController.

### Disconnect Timeout

The status manager tracks and reports a connection timeout. It calls function pointers into the ODriveContainer to handle pausing and resuming all ODrives.

### Error Handling

The ODriveController can detect and report the ODrive in an error mode. If this is the case, and the error is not considered unrecoverable, the ODriveController will attempt to clear the error and resume normal operation. The biggest case of this is the over-current error.

Unrecoverable errors will be reported to the status manager, and may be reset manually.

# Hardware

There are two hardware revisions of the ODrive REV module.
EDA files are available in the `hardware` folder.
See electrical documentation for more information/changelog.

-   [Revision 1](hardware/Odrive-REV-Module-Rev1-docs.md)

### Connections

Uses UART to communicate with the ODrive over the arduino pins in software serial.
Arduino Pins 7 and 8 are used for the UART communication with the ODrive.
Arduino Pin 7 TX -> ODrive RX
Arduino Pin 8 RX -> ODrive TX

## O Drive Configuration

The ODrive is not configured by the ROI module, it must be set up prior to use.
Ensure the ODrive is configured, and functions, then setup UART communication.

### UART Configuration

The ODrive must be configured to use UART communication.
The easiest way to do this is to use the ODrive Gui and connect to the ODrive over USB.

[GUI](https://gui.odriverobotics.com/dashboard)

In the Inspector tab, set the following parameters:

-   `odrv0.config.enable_uart_a = True`
-   `odrv0.config.uart_a_baudrate = 19200`
-   `odrv0.config.gpio6_mode = UART_A`
-   `odrv0.config.gpio7_mode = UART_A`

Then save the configuration and reboot the ODrive.
