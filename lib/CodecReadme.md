# Module Codec Lookup

Wiki for looking up the codec for each module. This is a reference for developers to understand the codec for each module. Used with the `ModuleCodec.h` file. Quick link to different modules and packet types:

Table of Contents

-   [sysAdmin](#sysadmin)
-   [GeneralGPIO](#generalgpio)
    -   [Module README](../Modules/GPIO%20ROI%20Module/README.md)
-   [ODrive](#odrive)

    -   [Module README](../Modules/ODrive%20ROI%20Module/README.md)

-   [Best Practices](#best-practices)

## sysAdmin

General actions that can be performed across all modules. Some modules may override responses to these actions, but they should all be able to respond to them.

SysAdmin Packets can have both metadata and action codes, but not subDeviceIDs. The metadata currently carries both additional packet information, such as wether it should be chained around the network and the reply octet. The action code determines the action to be performed. Note while a sysAdmin request can be sent to all modules in the chain, responses do not propagate through the chain, they are only sent back to the original sender, whose host address octet has been embedded in the second byte of metadata.

Options:

-   [Ping](#ping)
-   [Status Report](#status-report)
-   [Blacklist](#blacklist)

### Ping

Call a ping packet on the sysAdmin port with action code: `sysAdminConstants::PING`

A ping packet is a simple packet that is sent to the sysAdmin port to check if the client is there, and what it is.
No payload is required, and the response will be a pong packet.

#### Return

action code: `sysAdminConstants::PONG`

Payload [2 bytes]

-   0: Ready, 0 if not operable, 1 if operable.
-   1: Client type, see available options in `moduleTypesConstants` namespace.

### Status Report

Call a status report packet on the sysAdmin port with action code: `sysAdminConstants::STATUS_REPORT`

A status report packet is a packet that is sent to the sysAdmin port to check the status of the client. This works on every module.
No payload is required, and the response will be a status report packet.

#### Return

action code: `sysAdminConstants::STATUS_REPORT`

Payload [14 bytes]:

-   0: Status code, see available options in `statusReportConstants` namespace.
-   1: Time alive, hours
-   2: Time alive, minutes
-   3: Time alive, seconds
-   4: Supply voltage \* 100, high byte
-   5: Supply voltage \* 100, low byte
-   6: Type of client, see available options in `moduleTypesConstants` namespace.
-   7: Chain Neighbor Host Address Octet
-   8: Mac Address Octet 1
-   9: Mac Address Octet 2
-   10: Mac Address Octet 3
-   11: Mac Address Octet 4
-   12: Mac Address Octet 5
-   13: Mac Address Octet 6

### Blacklist

Call a blacklist packet on the sysAdmin port with action code: `sysAdminConstants::BLACKLIST`

This command can add remove or list blacklisted devices. The payload determines the action:

#### Payload

[2 Bytes]

-   0: Payload Action code, see available options in `blacklistConstants` namespace. `blacklistConstants::ADDBLACKLIST`, `blacklistConstants::REMOVEBLACKLIST`,`
-   1: Device octet to blacklist or remove from blacklist.

[1 Byte]

-   0: Action code, `blacklistConstants::LISTBLACKLIST`

#### Return

[1 Byte] `ADDBLACKLIST` or `REMOVEBLACKLIST`:

-   0: Success, 0 if not successful, 1 if successful.

[N Bytes] `LISTBLACKLIST`:

Each byte is a device octet that is blacklisted.

## GeneralGPIO

The most basic GPIO module, it can be used to set pin modes, read and write digital values, and read analog values.
See `GeneralGPIOConstants` for available action codes, and pin codes. The generalGPIO module subDeviceIDs are always pin codes.

Pin Codes:

-   Digital Pins (0-7, on the Arduino nano) are subDeviceIDs 0-7
-   Analog Pins (A0-A7, on the Arduino nano) are subDeviceIDs 10-17 Note: Analog pins 10-15 can be used as digital pins as well, but for simplicity, they always read analog values.

Options:

-   [Set Pin Mode](#set-pin-mode)
-   [Set Output](#set-output)
-   [Read](#read)

### Set Pin Mode

Call a set pin mode action on the GeneralGPIO port with action code: `GeneralGPIOConstants::SET_PIN_MODE`

The payload determines the pin mode:

-   INPUT: `GeneralGPIOConstants::INPUT_MODE`
-   INPUT_PULLUP: `GeneralGPIOConstants::INPUT_PULLUP_MODE`
-   OUTPUT: `GeneralGPIOConstants::OUTPUT_MODE`

#### Return

actionCode: `GeneralGPIOConstants::SET_PIN_MODE`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. Also returns 0 if the pin is not a valid pin.

### Set Output

Call a set output action on the GeneralGPIO port with action code: `GeneralGPIOConstants::SET_OUTPUT`
Note: The pin must be set to output mode before setting an output.

The payload determines the output value:
`GeneralGPIOConstants::LOW` or `GeneralGPIOConstants::HIGH`

#### Return

actionCode: `GeneralGPIOConstants::SET_OUTPUT`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. Also returns 0 if the pin is not a valid pin, or if the pin is not set to output mode.

### Read

Call a read digital action on the GeneralGPIO port with action code: `GeneralGPIOConstants::READ`
If the pin is a digital pin, ID 0-7, it will return the digital value of the pin, 0 or 1 in 1 byte.
If the pin is an analog pin, ID 10-17, it will return the analog value of the pin, 0-1023 in 2 bytes.

#### Return

actionCode: `GeneralGPIOConstants::READ`

Payload [1 byte]:

-   0: Value, 0 if low, 1 if high. For digital pins only.

Payload [2 bytes]:

-   0: Value high byte, 0-255 for analog pins only.
-   1: Value low byte, 0-3 for analog pins only.

## ODrive

The ODrive module is a module that interfaces with an ODrive motor controller, allowing for control of motors and encoders. It uses the UART protocol to communicate with the ODrive.

Options:

-   [Set Control Mode](#set-control-mode)
    -   [Get Control Mode](#get-control-mode)
-   [Set Input Mode](#set-input-mode)
    -   [Get Input Mode](#get-input-mode)
-   [Set Position](#set-position)
    -   [Get Position Set Point](#get-position-set-point)
    -   [Set Relative Position](#set-relative-position)
-   [Set Velocity](#set-velocity)
    -   [Get Velocity Set Point](#get-velocity-set-point)
-   [Set Torque](#set-torque)
    -   [Get Torque Set Point](#get-torque-set-point)
-   [Get Error](#get-error)
    -   [Clear Errors](#clear-errors)
-   Actual Values
    -   [Get Position](#get-position)
    -   [Get Velocity](#get-velocity)
    -   [Get Bus Voltage](#get-bus-voltage)
    -   [Get Current](#get-current)
    -   [Get FET Temperature](#get-fet-temperature)
    -   [Get Motor Temperature](#get-motor-temperature)
-   Glob Getters
    -   [Get Position and Velocity](#get-kinematic-feedback)
    -   [Get All](#get-all)

### Set Control Mode

Set the control mode of the ODrive, either target position, velocity, or torque. Use the code `ODriveConstants::SETCONTROLMODE` to call this.

Payloads[0]:

-   Position: `ODriveConstants::POSITIONMODE`
-   Velocity: `ODriveConstants::VELOCITYMODE`
-   Torque: `ODriveConstants::TORQUEMODE`

#### Return

actionCode: `ODriveConstants::SETCONTROLMODE`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to change control mode under error, or when motor is busy/moving)

### Get Control Mode

Get the control mode of the ODrive. Use the code `ODriveConstants::GETCONTROLMODE` to call this.

#### Return

actionCode: `ODriveConstants::GETCONTROLMODE`

Payload [1 byte]:

-   0: Control mode, see available options in `ODriveConstants` namespace.

## Set Input Mode

Sets the PID controller input on the ODrive. Use the code `ODriveConstants::SETINPUTMODE` to call this.

Payloads[1 Byte]:

-   `ODriveConstants::TRAP_TRAJ_MODE` : Trapezoidal trajectory mode (best acceleration mode for position control)
-   `ODriveConstants::POS_FILTER_MODE` : Position filter mode
-   `ODriveConstants::VEL_RAMP_MODE` : Velocity ramp mode (best acceleration mode for velocity control)
-   `ODriveConstants::TORQUE_RAMP_MODE` : Torque ramp mode (best acceleration mode for torque control)
-   `ODriveConstants::AUTO_BEST_FIT_MODE` : Automatically selects the best mode based on the control mode

#### Return

actionCode: `ODriveConstants::SETINPUTMODE`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to change input mode under error)

### Get Input Mode

Gets the PID controller input mode on the ODrive. Use the code `ODriveConstants::GETINPUTMODE` to call this.

#### Return

actionCode: `ODriveConstants::GETINPUTMODE`

Payload [1 byte]:

-   0: Input mode, see available options in `ODriveConstants` namespace.

### Set Position

Sets the target position when in position control mode. Use the code `ODriveConstants::SETPOSITION` to call this. When not in position control mode, this will have no effect.

Units: rev (revolutions)

Payload [4 bytes]: (float, big-endian)

-   0: Position float byte 1 (high byte)
-   1: Position float byte 2
-   2: Position float byte 3
-   3: Position float byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SETPOSITION`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to set position under error)

### Set Relative Position

Sets the target position relative to the current position when in position control mode. Use the code `ODriveConstants::SETRELATIVEPOSITION` to call this. When not in position control mode, this will have no effect.

Units: rev (revolutions)

Payload [4 bytes]: (float, big-endian)

-   0 : Position float byte 1 (high byte)
-   1 : Position float byte 2
-   2 : Position float byte 3
-   3 : Position float byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SETRELATIVEPOSITION`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to set relative position under error)

### Get Position Set Point

Gets the current position set point (not actual) of the motor. Use the code `ODriveConstants::GETPOSITIONSETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GETPOSITION`

Payload [4 bytes]: (float, big-endian)

-   0: Position float byte 1 (high byte)
-   1: Position float byte 2
-   2: Position float byte 3
-   3: Position float byte 4 (low byte)

### Set Velocity

Sets the target velocity when in velocity control mode, or the maximum velocity when in the other modes. Use the code `ODriveConstants::SETVELOCITY` to call this.

Units: rev/s (revolutions per second)

Payload [4 bytes]: (float, big-endian)

-   0: Velocity float byte 1 (high byte)
-   1: Velocity float byte 2
-   2: Velocity float byte 3
-   3: Velocity float byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SETVELOCITY`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to set velocity under error)

### Get Velocity Set Point

Gets the current velocity set point (not actual) value of the motor. Use the code `ODriveConstants::GETVELOCITYSETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GETVELOCITYSETPOINT`

Payload [4 bytes]: (float, big-endian)

-   0: Velocity float byte 1 (high byte)
-   1: Velocity float byte 2
-   2: Velocity float byte 3
-   3: Velocity float byte 4 (low byte)

### Set Torque

Sets the target torque when in torque control mode, or the maximum torque when in the other modes. Use the code `ODriveConstants::SETTORQUE` to call this.

Unit: Nm (Newton meters)

Payload [4 bytes]: (float, big-endian)

-   0: Torque float byte 1 (high byte)
-   1: Torque float byte 2
-   2: Torque float byte 3
-   3: Torque float byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SETTORQUE`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to set torque under error)

### Get Torque Set Point

Gets the current torque set point (not actual) value of the motor. Use the code `ODriveConstants::GETTORQUESETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GETTORQUESETPOINT`

Payload [4 bytes]: (float, big-endian)

-   0: Torque float byte 1 (high byte)
-   1: Torque float byte 2
-   2: Torque float byte 3
-   3: Torque float byte 4 (low byte)

### Get Error

Gets the error code of the ODrive. Use the code `ODriveConstants::GETERROR` to call this.

#### Return

actionCode: `ODriveConstants::GETERROR`

Payload [4 bytes]: (uint32_t, big-endian)

-   0: Error code long byte 1 (high byte)
-   1: Error code long byte 2
-   2: Error code long byte 3
-   3: Error code long byte 4 (low byte)

### Clear Errors

Clears the error code of the ODrive. Use the code `ODriveConstants::CLEARERRORS` to call this.

#### Return

actionCode: `ODriveConstants::CLEARERRORS`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful.

### Get Position

Gets the current position (real) of the motor. Use the code `ODriveConstants::GETPOSITION` to call this.

#### Return

actionCode: `ODriveConstants::GETPOSITION`

Payload [4 bytes]: (float, big-endian)

-   0: Position float byte 1 (high byte)
-   1: Position float byte 2
-   2: Position float byte 3
-   3: Position float byte 4 (low byte)

### Get Velocity

Gets the current velocity (real) of the motor. Use the code `ODriveConstants::GETVELOCITY` to call this.

#### Return

actionCode: `ODriveConstants::GETVELOCITY`

Payload [4 bytes]: (float, big-endian)

-   0: Velocity float byte 1 (high byte)
-   1: Velocity float byte 2
-   2: Velocity float byte 3
-   3: Velocity float byte 4 (low byte)

### Get Bus Voltage

Returns the bus voltage of the ODrive. Use the code `ODriveConstants::GETBUSVOLTAGE` to call this.

#### Return

actionCode: `ODriveConstants::GETBUSVOLTAGE`

Payload [4 bytes]: (float, big-endian)

-   0: Bus voltage float byte 1 (high byte)
-   1: Bus voltage float byte 2
-   2: Bus voltage float byte 3
-   3: Bus voltage float byte 4 (low byte)

### Get Current

Returns the current of the ODrive. Use the code `ODriveConstants::GETCURRENT` to call this.

#### Return

actionCode: `ODriveConstants::GETCURRENT`

Payload [4 bytes]: (float, big-endian)

-   0: Current float byte 1 (high byte)
-   1: Current float byte 2
-   2: Current float byte 3
-   3: Current float byte 4 (low byte)

### Get FET Temperature

Returns the FET temperature of the ODrive. Use the code `ODriveConstants::GETFETTEMPERATURE` to call this.

#### Return

actionCode: `ODriveConstants::GETFETTEMPERATURE`

Payload [4 bytes]: (float, big-endian)

-   0: FET temperature float byte 1 (high byte)
-   1: FET temperature float byte 2
-   2: FET temperature float byte 3
-   3: FET temperature float byte 4 (low byte)

### Get Motor Temperature

Returns the motor temperature of the ODrive. Use the code `ODriveConstants::GETMOTORTEMPERATURE` to call this.

#### Return

actionCode: `ODriveConstants::GETMOTORTEMPERATURE`

Payload [4 bytes]: (float, big-endian)

-   0: Motor temperature float byte 1 (high byte)
-   1: Motor temperature float byte 2
-   2: Motor temperature float byte 3
-   3: Motor temperature float byte 4 (low byte)

### Get Kinematic Feedback

Returns the position and velocity of the motor. Use the code `ODriveConstants::GETKINEMATICFEEDBACK` to call this.

#### Return

actionCode: `ODriveConstants::GETKINEMATICFEEDBACK`

Payload [8 bytes]: (float, big-endian)

-   0: Position float byte 1 (high byte)
-   1: Position float byte 2
-   2: Position float byte 3
-   3: Position float byte 4 (low byte)
-   4: Velocity float byte 1 (high byte)
-   5: Velocity float byte 2
-   6: Velocity float byte 3
-   7: Velocity float byte 4 (low byte)

### Get All

Returns all the values of the ODrive. Use the code `ODriveConstants::GETALL` to call this.

#### Return

actionCode: `ODriveConstants::GETALL`

Payload [28 bytes]:

-   0: Position float byte 1 (high byte)
-   1: Position float byte 2
-   2: Position float byte 3
-   3: Position float byte 4 (low byte)

-   4: Velocity float byte 1 (high byte)
-   5: Velocity float byte 2
-   6: Velocity float byte 3
-   7: Velocity float byte 4 (low byte)

-   8: Voltage float byte 1 (high byte)
-   9: Voltage float byte 2
-   10: Voltage float byte 3
-   11: Voltage float byte 4 (low byte)

-   12: Current float byte 1 (high byte)
-   13: Current float byte 2
-   14: Current float byte 3
-   15: Current float byte 4 (low byte)

-   16: FET temperature float byte 1 (high byte)
-   17: FET temperature float byte 2
-   18: FET temperature float byte 3
-   19: FET temperature float byte 4 (low byte)

-   20: Motor temperature float byte 1 (high byte)
-   21: Motor temperature float byte 2
-   22: Motor temperature float byte 3
-   23: Motor temperature float byte 4 (low byte)

-   24: Error code long byte 1 (high byte)
-   25: Error code long byte 2
-   26: Error code long byte 3
-   27: Error code long byte 4 (low byte)

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
