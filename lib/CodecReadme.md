# Module Codec Lookup

Wiki for looking up the codec for each module. This is a reference for developers to understand the codec for each module. Used with the `ModuleCodec.h` file. Quick link to different modules and packet types:

- [sysAdmin](#sysAdmin)
- [GeneralGPIO](#GeneralGPIO)

  - [General GPIO Module README](../Modules/GPIO%20ROI%20Module/README.md)

- [Best Practices](#best-practices)

## sysAdmin

General actions that can be performed across all modules. Some modules may override responses to these actions, but they should all be able to respond to them.

SysAdmin Packets can have both metadata and action codes, but not subDeviceIDs. The metadata currently carries both additional packet information, such as wether it should be chained around the network and the reply octet. The action code determines the action to be performed. Note while a sysAdmin request can be sent to all modules in the chain, responses do not propagate through the chain, they are only sent back to the original sender, whose host address octet has been embedded in the second byte of metadata.

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

Call a blacklist packet on the sysAdmin port with action code: `sysAdminConstants::BLACKLIST`

This command can add remove or list blacklisted devices. The payload determines the action:

#### Payload

[2 Bytes]

- 0: Payload Action code, see available options in `blacklistConstants` namespace. `blacklistConstants::ADDBLACKLIST`, `blacklistConstants::REMOVEBLACKLIST`,`
- 1: Device octet to blacklist or remove from blacklist.

[1 Byte]

- 0: Action code, `blacklistConstants::LISTBLACKLIST`

#### Return

[1 Byte] `ADDBLACKLIST` or `REMOVEBLACKLIST`:

- 0: Success, 0 if not successful, 1 if successful.

[N Bytes] `LISTBLACKLIST`:

Each byte is a device octet that is blacklisted.

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

## ODrive

The ODrive module is a module that interfaces with an ODrive motor controller, allowing for control of motors and encoders. It uses the UART protocol to communicate with the ODrive.

### Set Control Mode

Set the control mode of the ODrive, either target position, velocity, or torque. Use the code `ODriveConstants::SETCONTROLMODE` to call this.

Payloads[0]:

- Position: `ODriveConstants::POSITIONMODE`
- Velocity: `ODriveConstants::VELOCITYMODE`
- Torque: `ODriveConstants::TORQUEMODE`

#### Return

actionCode: `ODriveConstants::SETCONTROLMODE`

Payload [1 byte]:

- 0: Success, 0 if not successful, 1 if successful. (Unable to change control mode under error, or when motor is busy/moving)

### Get Control Mode

Get the control mode of the ODrive. Use the code `ODriveConstants::GETCONTROLMODE` to call this.

#### Return

actionCode: `ODriveConstants::GETCONTROLMODE`

Payload [1 byte]:

- 0: Control mode, see available options in `ODriveConstants` namespace.

## Set Input Mode

Sets the PID controller input on the ODrive. Use the code `ODriveConstants::SETINPUTMODE` to call this.

Payloads[1 Byte]:

- `ODriveConstants::TRAP_TRAJ_MODE` : Trapezoidal trajectory mode (best acceleration mode for position control)
- `ODriveConstants::POS_FILTER_MODE` : Position filter mode
- `ODriveConstants::VEL_RAMP_MODE` : Velocity ramp mode (best acceleration mode for velocity control)
- `ODriveConstants::TORQUE_RAMP_MODE` : Torque ramp mode (best acceleration mode for torque control)
- `ODriveConstants::AUTO_BEST_FIT_MODE` : Automatically selects the best mode based on the control mode

#### Return

actionCode: `ODriveConstants::SETINPUTMODE`

Payload [1 byte]:

- 0: Success, 0 if not successful, 1 if successful. (Unable to change input mode under error)

### Set Position

Sets the target position when in position control mode. Use the code `ODriveConstants::SETPOSITION` to call this. When not in position control mode, this will have no effect.

Units: rev (revolutions)

Payload [4 bytes]: (float, big-endian)

- 0: Position long byte 1 (high byte)
- 1: Position long byte 2
- 2: Position long byte 3
- 3: Position long byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SETPOSITION`

Payload [1 byte]:

- 0: Success, 0 if not successful, 1 if successful. (Unable to set position under error)

### Get Position Set Point

Gets the current position set point (not actual) of the motor. Use the code `ODriveConstants::GETPOSITIONSETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GETPOSITION`

Payload [4 bytes]: (float, big-endian)

- 0: Position long byte 1 (high byte)
- 1: Position long byte 2
- 2: Position long byte 3
- 3: Position long byte 4 (low byte)

### Set Velocity

Sets the target velocity when in velocity control mode, or the maximum velocity when in the other modes. Use the code `ODriveConstants::SETVELOCITY` to call this.

Units: rev/s (revolutions per second)

Payload [4 bytes]: (float, big-endian)

- 0: Velocity long byte 1 (high byte)
- 1: Velocity long byte 2
- 2: Velocity long byte 3
- 3: Velocity long byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SETVELOCITY`

Payload [1 byte]:

- 0: Success, 0 if not successful, 1 if successful. (Unable to set velocity under error)

### Get Velocity Set Point

Gets the current velocity set point (not actual) value of the motor. Use the code `ODriveConstants::GETVELOCITYSETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GETVELOCITYSETPOINT`

Payload [4 bytes]: (float, big-endian)

- 0: Velocity long byte 1 (high byte)
- 1: Velocity long byte 2
- 2: Velocity long byte 3
- 3: Velocity long byte 4 (low byte)

### Set Torque

Sets the target torque when in torque control mode, or the maximum torque when in the other modes. Use the code `ODriveConstants::SETTORQUE` to call this.

Unit: Nm (Newton meters)

Payload [4 bytes]: (float, big-endian)

- 0: Torque long byte 1 (high byte)
- 1: Torque long byte 2
- 2: Torque long byte 3
- 3: Torque long byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SETTORQUE`

Payload [1 byte]:

- 0: Success, 0 if not successful, 1 if successful. (Unable to set torque under error)

### Get Torque Set Point

Gets the current torque set point (not actual) value of the motor. Use the code `ODriveConstants::GETTORQUESETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GETTORQUESETPOINT`

Payload [4 bytes]: (float, big-endian)

- 0: Torque long byte 1 (high byte)
- 1: Torque long byte 2
- 2: Torque long byte 3
- 3: Torque long byte 4 (low byte)

### Get Error

Gets the error code of the ODrive. Use the code `ODriveConstants::GETERROR` to call this.

#### Return

actionCode: `ODriveConstants::GETERROR`

Payload [4 bytes]: (uint32_t, big-endian)

- 0: Error code long byte 1 (high byte)
- 1: Error code long byte 2
- 2: Error code long byte 3
- 3: Error code long byte 4 (low byte)

### Clear Errors

Clears the error code of the ODrive. Use the code `ODriveConstants::CLEARERRORS` to call this.

#### Return

actionCode: `ODriveConstants::CLEARERRORS`

Payload [1 byte]:

- 0: Success, 0 if not successful, 1 if successful.

### Get Position

Gets the current position (real) of the motor. Use the code `ODriveConstants::GETPOSITION` to call this.

#### Return

actionCode: `ODriveConstants::GETPOSITION`

Payload [4 bytes]: (float, big-endian)

- 0: Position long byte 1 (high byte)
- 1: Position long byte 2
- 2: Position long byte 3
- 3: Position long byte 4 (low byte)

### Get Velocity

Gets the current velocity (real) of the motor. Use the code `ODriveConstants::GETVELOCITY` to call this.

#### Return

actionCode: `ODriveConstants::GETVELOCITY`

Payload [4 bytes]: (float, big-endian)

- 0: Velocity long byte 1 (high byte)
- 1: Velocity long byte 2
- 2: Velocity long byte 3
- 3: Velocity long byte 4 (low byte)

### Get Bus Voltage

Returns the bus voltage of the ODrive. Use the code `ODriveConstants::GETBUSVOLTAGE` to call this.

#### Return

actionCode: `ODriveConstants::GETBUSVOLTAGE`

Payload [2 bytes]: (Voltage \* 10, big-endian) [164 = 16.4V]

- 0: Bus voltage high byte
- 1: Bus voltage low byte

### Get Current

Returns the current of the ODrive. Use the code `ODriveConstants::GETCURRENT` to call this.

#### Return

actionCode: `ODriveConstants::GETCURRENT`

Payload [4 bytes]: (float, big-endian)

- 0: Current long byte 1 (high byte)
- 1: Current long byte 2
- 2: Current long byte 3
- 3: Current long byte 4 (low byte)

### Get Temperature

Returns the temperature of the ODrive, and if applicable, the motor. Use the code `ODriveConstants::GETTEMPERATURE` to call this.

#### Return

actionCode: `ODriveConstants::GETTEMPERATURE`

Payload [4 bytes]: (temp C, big-endian) [250 = 250C]

- 0: ODrive FET temperature high byte
- 1: ODrive FET temperature low byte
- 2: Motor temperature high byte
- 3: Motor temperature low byte

## Best Practices

1. Encapsulate your code in a namespace. Keep constants in a separate namespace, and use `constexpr` rather than `const` for them. This will help prevent name collisions and make your code more readable. `Constexpr` is also more efficient than `const`, as it is evaluated at compile time rather than run time. (Like `#define` but with type and scope safety)

   - Example:

   ```cpp
   #ifndef MY_MODULE_H
   #define MY_MODULE_H

   namespace myModuleConstants {
       constexpr int MY_CONSTANT = 42; // Note: constexpr rather than const
   }

   #endif
   ```

2. Descriptive type names are preferred over numbers. This makes it easier to understand the purpose of constant definitions. For example, use `GeneralGPIOConstants::INPUT_MODE` rather than `0`.

   - Example:

   ```cpp
    typedef uint16_t actionConstant; //These are predefined types for the sake of clarity (They are not namespaced)
    typedef uint16_t payloadConstant; //You may need to define your own types, please keep them within the module namespace

   namespace GeneralGPIOConstants {
       constexpr uint16_t INPUT_MODE = 0; // uint16_t makes it unclear the purpose of the constant (This is a payload value)
       constexpr uint16_t SET_PIN_MODE = 1; // This is an action code
       constexpr uint16_t OUTPUT_MODE = 2; // This is a payload value
   }

   namespace BetterGeneralGPIOConstants {
       constexpr actionConstant SET_PIN_MODE = 1; // This is an action code

       constexpr payloadConstant INPUT_MODE = 0; // uint16_t makes it unclear the purpose of the constant (This is a payload value)
       constexpr payloadConstant OUTPUT_MODE = 2; // This is a payload value
   }
   ```
