# ODrive Module

### Hardware

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

- `odrv0.config.enable_uart_a = True`
- `odrv0.config.uart_a_baudrate = 19200`
- `odrv0.config.gpio6_mode = UART_A`
- `odrv0.config.gpio7_mode = UART_A`

Then save the configuration and reboot the ODrive.
