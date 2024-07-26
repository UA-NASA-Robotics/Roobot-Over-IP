# Module Codec Lookup

Wiki for looking up the codec for each module. This is a reference for developers to understand the codec for each module. Used with the `ModuleCodec.h` file. Quick link to different modules and packet types:

- [sysAdmin](#sysAdmin)
- [GeneralGPIO](#GeneralGPIO)

## sysAdmin

General actions that can be performed across all modules. Some modules may override responses to these actions, but they should all be able to respond to them.

SysAdmin Packets can have both metadata and action codes, but not subDeviceIDs. The metadata currently carries both additional packet information, such as wether it should be chained around the network. The action code determines the action to be performed. Note while a sysAdmin request can be sent to all modules in the chain, responses do not propagate through the chain, they are only sent back to the original sender.

### Ping

Call a ping packet on the sysAdmin port with action code: `sysAdminConstants::PING`

A ping packet is a simple packet that is sent to the sysAdmin port to check if the client is there, and what it is.
No payload is required, and the response will be a pong packet.

#### Return

action code: `sysAdminConstants::PONG`

Payload [2 bytes]

- 0: Ready, 0 if not operable, 1 if operable.
- 1: Client type, see available options in `moduleTypesConstants` namespace.

### Status Report

Call a status report packet on the sysAdmin port with action code: `sysAdminConstants::STATUS_REPORT`

A status report packet is a packet that is sent to the sysAdmin port to check the status of the client. This works on every module.
No payload is required, and the response will be a status report packet.

#### Return

action code: `sysAdminConstants::STATUS_REPORT`

Payload [14 bytes]:

- 0: Status code, see available options in `statusReportConstants` namespace.
- 1: Time alive, hours
- 2: Time alive, minutes
- 3: Time alive, seconds
- 4: Supply voltage \* 100, high byte
- 5: Supply voltage \* 100, low byte
- 6: Type of client, see available options in `moduleTypesConstants` namespace.
- 7: Chain Neighbor Host Address Octet
- 8: Mac Address Octet 1
- 9: Mac Address Octet 2
- 10: Mac Address Octet 3
- 11: Mac Address Octet 4
- 12: Mac Address Octet 5
- 13: Mac Address Octet 6

### Blacklist

tbd

## GeneralGPIO

The most basic GPIO module, it can be used to set pin modes, read and write digital values, and read analog values.
See `GeneralGPIOConstants` for available action codes, and pin codes. The generalGPIO module subDeviceIDs are always pin codes.

Pin Codes:

- Digital Pins (0-7, on the Arduino nano) are subDeviceIDs 0-7
- Analog Pins (A0-A7, on the Arduino nano) are subDeviceIDs 10-17 Note: Analog pins 10-15 can be used as digital pins as well, but for simplicity, they always read analog values.

### Set Pin Mode

Call a set pin mode action on the GeneralGPIO port with action code: `GeneralGPIOConstants::SET_PIN_MODE`

The payload determines the pin mode:

- INPUT: `GeneralGPIOConstants::INPUT_MODE`
- INPUT_PULLUP: `GeneralGPIOConstants::INPUT_PULLUP_MODE`
- OUTPUT: `GeneralGPIOConstants::OUTPUT_MODE`

#### Return

actionCode: `GeneralGPIOConstants::SET_PIN_MODE`

Payload [1 byte]:

- 0: Success, 0 if not successful, 1 if successful. Also returns 0 if the pin is not a valid pin.

### Set Output

Call a set output action on the GeneralGPIO port with action code: `GeneralGPIOConstants::SET_OUTPUT`
Note: The pin must be set to output mode before setting an output.

The payload determines the output value:
`GeneralGPIOConstants::LOW` or `GeneralGPIOConstants::HIGH`

#### Return

actionCode: `GeneralGPIOConstants::SET_OUTPUT`

Payload [1 byte]:

- 0: Success, 0 if not successful, 1 if successful. Also returns 0 if the pin is not a valid pin, or if the pin is not set to output mode.

### Read

Call a read digital action on the GeneralGPIO port with action code: `GeneralGPIOConstants::READ`
If the pin is a digital pin, ID 0-7, it will return the digital value of the pin, 0 or 1 in 1 byte.
If the pin is an analog pin, ID 10-17, it will return the analog value of the pin, 0-1023 in 2 bytes.

#### Return

actionCode: `GeneralGPIOConstants::READ`

Payload [1 byte]:

- 0: Value, 0 if low, 1 if high. For digital pins only.

Payload [2 bytes]:

- 0: Value high byte, 0-255 for analog pins only.
- 1: Value low byte, 0-3 for analog pins only.
