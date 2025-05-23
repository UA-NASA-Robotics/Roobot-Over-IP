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

Set the control mode of the ODrive, either target position, velocity, or torque. Use the code `ODriveConstants::SET_CONTROL_MODE` to call this.

Payloads[0]:

-   Position: `ODriveConstants::POSITION_MODE`
-   Velocity: `ODriveConstants::VELOCITY_MODE`
-   Torque: `ODriveConstants::TORQUE_MODE`

#### Return

actionCode: `ODriveConstants::SET_CONTROL_MODE`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to change control mode under error, or when motor is busy/moving)

### Get Control Mode

Get the control mode of the ODrive. Use the code `ODriveConstants::GET_CONTROL_MODE` to call this.

#### Return

actionCode: `ODriveConstants::GET_CONTROL_MODE`

Payload [1 byte]:

-   0: Control mode, see available options in `ODriveConstants` namespace.

### Set Input Mode

Sets the PID controller input on the ODrive. Use the code `ODriveConstants::SET_INPUT_MODE` to call this.

Payloads[1 Byte]:

-   `ODriveConstants::TRAP_TRAJ_MODE` : Trapezoidal trajectory mode (best acceleration mode for position control)
-   `ODriveConstants::POS_FILTER_MODE` : Position filter mode
-   `ODriveConstants::VEL_RAMP_MODE` : Velocity ramp mode (best acceleration mode for velocity control)
-   `ODriveConstants::TORQUE_RAMP_MODE` : Torque ramp mode (best acceleration mode for torque control)
-   `ODriveConstants::AUTO_BEST_FIT_MODE` : Automatically selects the best mode based on the control mode

#### Return

actionCode: `ODriveConstants::SET_INPUT_MODE`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to change input mode under error)

### Get Input Mode

Gets the PID controller input mode on the ODrive. Use the code `ODriveConstants::GET_INPUT_MODE` to call this.

#### Return

actionCode: `ODriveConstants::GET_INPUT_MODE`

Payload [1 byte]:

-   0: Input mode, see available options in `ODriveConstants` namespace.

### Set Position

Sets the target position when in position control mode. Use the code `ODriveConstants::SET_POSITION` to call this. When not in position control mode, this will have no effect.

Units: rev (revolutions)

Payload [4 bytes]: (float, big-endian)

-   0: Position float byte 1 (high byte)
-   1: Position float byte 2
-   2: Position float byte 3
-   3: Position float byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SET_POSITION`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to set position under error)

### Set Relative Position

Sets the target position relative to the current position when in position control mode. Use the code `ODriveConstants::SET_RELATIVE_POSITION` to call this. When not in position control mode, this will have no effect.

Units: rev (revolutions)

Payload [4 bytes]: (float, big-endian)

-   0 : Position float byte 1 (high byte)
-   1 : Position float byte 2
-   2 : Position float byte 3
-   3 : Position float byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SET_RELATIVE_POSITION`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to set relative position under error)

### Get Position Set Point

Gets the current position set point (not actual) of the motor. Use the code `ODriveConstants::GET_POSITION_SETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GET_POSITION`

Payload [4 bytes]: (float, big-endian)

-   0: Position float byte 1 (high byte)
-   1: Position float byte 2
-   2: Position float byte 3
-   3: Position float byte 4 (low byte)

### Set Velocity

Sets the target velocity when in velocity control mode, or the maximum velocity when in the other modes. Use the code `ODriveConstants::SET_VELOCITY` to call this.

Units: rev/s (revolutions per second)

Payload [4 bytes]: (float, big-endian)

-   0: Velocity float byte 1 (high byte)
-   1: Velocity float byte 2
-   2: Velocity float byte 3
-   3: Velocity float byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SET_VELOCITY`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to set velocity under error)

### Get Velocity Set Point

Gets the current velocity set point (not actual) value of the motor. Use the code `ODriveConstants::GET_VELOCITY_SETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GET_VELOCITY_SETPOINT`

Payload [4 bytes]: (float, big-endian)

-   0: Velocity float byte 1 (high byte)
-   1: Velocity float byte 2
-   2: Velocity float byte 3
-   3: Velocity float byte 4 (low byte)

### Set Torque

Sets the target torque when in torque control mode, or the maximum torque when in the other modes. Use the code `ODriveConstants::SET_TORQUE` to call this.

Unit: Nm (Newton meters)

Payload [4 bytes]: (float, big-endian)

-   0: Torque float byte 1 (high byte)
-   1: Torque float byte 2
-   2: Torque float byte 3
-   3: Torque float byte 4 (low byte)

#### Return

actionCode: `ODriveConstants::SET_TORQUE`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful. (Unable to set torque under error)

### Get Torque Set Point

Gets the current torque set point (not actual) value of the motor. Use the code `ODriveConstants::GET_TORQUE_SETPOINT` to call this.

#### Return

actionCode: `ODriveConstants::GET_TORQUE_SETPOINT`

Payload [4 bytes]: (float, big-endian)

-   0: Torque float byte 1 (high byte)
-   1: Torque float byte 2
-   2: Torque float byte 3
-   3: Torque float byte 4 (low byte)

### Get Error

Gets the error code of the ODrive. Use the code `ODriveConstants::GET_ERROR` to call this.

#### Return

actionCode: `ODriveConstants::GET_ERROR`

Payload [4 bytes]: (uint32_t, big-endian)

-   0: Error code long byte 1 (high byte)
-   1: Error code long byte 2
-   2: Error code long byte 3
-   3: Error code long byte 4 (low byte)

### Clear Errors

Clears the error code of the ODrive. Use the code `ODriveConstants::CLEAR_ERRORS` to call this.

#### Return

actionCode: `ODriveConstants::CLEAR_ERRORS`

Payload [1 byte]:

-   0: Success, 0 if not successful, 1 if successful.

### Get Position

Gets the current position (real) of the motor. Use the code `ODriveConstants::GET_POSITION` to call this.

#### Return

actionCode: `ODriveConstants::GET_POSITION`

Payload [4 bytes]: (float, big-endian)

-   0: Position float byte 1 (high byte)
-   1: Position float byte 2
-   2: Position float byte 3
-   3: Position float byte 4 (low byte)

### Get Velocity

Gets the current velocity (real) of the motor. Use the code `ODriveConstants::GET_VELOCITY` to call this.

#### Return

actionCode: `ODriveConstants::GET_VELOCITY`

Payload [4 bytes]: (float, big-endian)

-   0: Velocity float byte 1 (high byte)
-   1: Velocity float byte 2
-   2: Velocity float byte 3
-   3: Velocity float byte 4 (low byte)

### Get Bus Voltage

Returns the bus voltage of the ODrive. Use the code `ODriveConstants::GET_BUS_VOLTAGE` to call this.

#### Return

actionCode: `ODriveConstants::GET_BUS_VOLTAGE`

Payload [4 bytes]: (float, big-endian)

-   0: Bus voltage float byte 1 (high byte)
-   1: Bus voltage float byte 2
-   2: Bus voltage float byte 3
-   3: Bus voltage float byte 4 (low byte)

### Get Current

Returns the current of the ODrive. Use the code `ODriveConstants::GET_CURRENT` to call this.

#### Return

actionCode: `ODriveConstants::GET_CURRENT`

Payload [4 bytes]: (float, big-endian)

-   0: Current float byte 1 (high byte)
-   1: Current float byte 2
-   2: Current float byte 3
-   3: Current float byte 4 (low byte)

### Get FET Temperature

Returns the FET temperature of the ODrive. Use the code `ODriveConstants::GET_FET_TEMPERATURE` to call this.

#### Return

actionCode: `ODriveConstants::GET_FET_TEMPERATURE`

Payload [4 bytes]: (float, big-endian)

-   0: FET temperature float byte 1 (high byte)
-   1: FET temperature float byte 2
-   2: FET temperature float byte 3
-   3: FET temperature float byte 4 (low byte)

### Get Motor Temperature

Returns the motor temperature of the ODrive. Use the code `ODriveConstants::GET_MOTOR_TEMPERATURE` to call this.

#### Return

actionCode: `ODriveConstants::GET_MOTOR_TEMPERATURE`

Payload [4 bytes]: (float, big-endian)

-   0: Motor temperature float byte 1 (high byte)
-   1: Motor temperature float byte 2
-   2: Motor temperature float byte 3
-   3: Motor temperature float byte 4 (low byte)

### Get Kinematic Feedback

Returns the position and velocity of the motor. Use the code `ODriveConstants::GET_KINEMATIC_FEEDBACK` to call this.

#### Return

actionCode: `ODriveConstants::GET_KINEMATIC_FEEDBACK`

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

Returns all the values of the ODrive. Use the code `ODriveConstants::GET_ALL` to call this.

#### Return

actionCode: `ODriveConstants::GET_ALL`

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
