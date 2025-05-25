## Actuator

The actuator module is a component interface between ROS and a variety of linear actuators. It can be configured by main.cpp to accept different encoders, drive methods, and more parameters.

Options:

-   [Set Relative Length Target](#set-relative-length-target)
-   [Set Absolute Length Target](#set-absolute-length-target)
-   [Get Target Length](#get-target-length)
-   [Get Current Length](#get-current-length)
-   [Set Velocity](#set-velocity)
-   [Get Target Velocity](#get-target-velocity)
-   [Get Last Home](#get-last-home)
-   [Set Control Mode](#set-control-mode)
-   [Get Control Mode](#get-control-mode)
-   PID Tuning:
    -   [Set Speed PID Constants](#set-speed-pid-constants)
    -   [Get Speed PID Constants](#get-speed-pid-constants)
    -   [Set Length PID Constants](#set-length-pid-constants)
    -   [Get Length PID Constants](#get-length-pid-constants)

### Set Relative Length Target

Sets the target length of the actuator relative to its current position. This is useful for moving the actuator a specific distance without needing to know its absolute position.
Use the `ActuatorConstants::SET_RELATIVE_LENGTH_TARGET` to call this

Payload[2]:

-   0: uint16_t high byte of the target offset in mm
-   1: uint16_t low byte of the target offset in mm

#### Returns

Payload[0]: (NA)

### Set Absolute Length Target

Sets the target length of the actuator to an absolute position. This is useful for moving the actuator to a specific position in the workspace.
Use the `ActuatorConstants::SET_ABSOLUTE_LENGTH_TARGET` to call this

Payload[2]:

-   0: uint16_t high byte of the target offset in mm
-   1: uint16_t low byte of the target offset in mm

#### Returns

Payload[0]: (NA)

### Get Target Length

Gets the target length of the actuator. This is useful for checking the current target position of the actuator.
Use the `ActuatorConstants::GET_TARGET_LENGTH` to call this

Payload[0]

#### Returns

Payload[2]:

-   0: uint16_t high byte of the target offset in mm
-   1: uint16_t low byte of the target offset in mm

### Get Current Length

Gets the current length of the actuator. This is useful for checking the current position of the actuator.
Use the `ActuatorConstants::GET_CURRENT_LENGTH` to call this

Payload[0]

#### Returns

Payload[2]:

-   0: uint16_t high byte of the current offset in mm
-   1: uint16_t low byte of the current offset in mm

### Set Velocity

Sets the velocity of the actuator. This is useful for controlling the speed at which the actuator moves.
Use the `ActuatorConstants::SET_VELOCITY` to call this

Payload[4]:

-   0: float32 high byte of the velocity in mm/s
-   1: float32 mid byte of the velocity in mm/s
-   2: float32 mid byte of the velocity in mm/s
-   3: float32 low byte of the velocity in mm/s

Note: when using IBT2 motor driver, the velocity is always maximum. Any positive value will be forward, and any negative value will be backward.

#### Returns

Payload[0]: (NA)

### Get Target Velocity

Gets the target velocity of the actuator. This is useful for checking the current target speed of the actuator.
Use the `ActuatorConstants::GET_TARGET_VELOCITY` to call this

Payload[0]

#### Returns

Payload[4]:

-   0: float32 high byte of the velocity in mm/s
-   1: float32 mid byte of the velocity in mm/s
-   2: float32 mid byte of the velocity in mm/s
-   3: float32 low byte of the velocity in mm/s

### Get Last Home

Gets the last home position of the actuator. This is useful for checking the last known home position of the actuator.
Use the `ActuatorConstants::GET_LAST_HOME` to call this
Payload[0]

#### Returns

Payload[4]:

-   0: uint32_t high byte of the last home time in ms
-   1: uint32_t mid byte of the last home time in ms
-   2: uint32_t mid byte of the last home time in ms
-   3: uint32_t low byte of the last home time in ms

### Set Control Mode

Sets the control mode of the actuator. This is useful for switching between different control modes, such as position control or velocity control.
Use the `ActuatorConstants::SET_CONTROL_MODE` to call this

Payload[1]:

-   0: uint8_t either `ActuatorConstants::LENGTH_MODE` or `ActuatorConstants::VELOCITY_MODE`

#### Returns

Payload[0]: (NA)

### Get Control Mode

Gets the control mode of the actuator. This is useful for checking the current control mode of the actuator.
Use the `ActuatorConstants::GET_CONTROL_MODE` to call this

Payload[0]

#### Returns

Payload[1]:

-   0: uint8_t either `ActuatorConstants::LENGTH_MODE` or `ActuatorConstants::VELOCITY_MODE`

### Set Speed PID Constants

Sets the PID constants for the speed control of the actuator. This is useful for tuning the speed control loop of the actuator.
Use the `ActuatorConstants::SET_SPEED_PID` to call this

Payload[12]:

-   0: float32 high byte of the proportional constant
-   1: float32 mid byte of the proportional constant
-   2: float32 mid byte of the proportional constant
-   3: float32 low byte of the proportional constant
-   4: float32 high byte of the integral constant
-   5: float32 mid byte of the integral constant
-   6: float32 mid byte of the integral constant
-   7: float32 low byte of the integral constant
-   8: float32 high byte of the derivative constant
-   9: float32 mid byte of the derivative constant
-   10: float32 mid byte of the derivative constant
-   11: float32 low byte of the derivative constant

#### Returns

Payload[0]: (NA)

### Get Speed PID Constants

Gets the PID constants for the speed control of the actuator. This is useful for checking the current PID constants of the actuator.
Use the `ActuatorConstants::GET_SPEED_PID` to call this

Payload[0]

#### Returns

Payload[12]:

-   0: float32 high byte of the proportional constant
-   1: float32 mid byte of the proportional constant
-   2: float32 mid byte of the proportional constant
-   3: float32 low byte of the proportional constant
-   4: float32 high byte of the integral constant
-   5: float32 mid byte of the integral constant
-   6: float32 mid byte of the integral constant
-   7: float32 low byte of the integral constant
-   8: float32 high byte of the derivative constant
-   9: float32 mid byte of the derivative constant
-   10: float32 mid byte of the derivative constant
-   11: float32 low byte of the derivative constant

### Set Length PID Constants

Sets the PID constants for the length control of the actuator. This is useful for tuning the length control loop of the actuator.
Use the `ActuatorConstants::SET_LENGTH_PID` to call this

Payload[12]:

-   0: float32 high byte of the proportional constant
-   1: float32 mid byte of the proportional constant
-   2: float32 mid byte of the proportional constant
-   3: float32 low byte of the proportional constant
-   4: float32 high byte of the integral constant
-   5: float32 mid byte of the integral constant
-   6: float32 mid byte of the integral constant
-   7: float32 low byte of the integral constant
-   8: float32 high byte of the derivative constant
-   9: float32 mid byte of the derivative constant
-   10: float32 mid byte of the derivative constant
-   11: float32 low byte of the derivative constant

#### Returns

Payload[0]: (NA)

### Get Length PID Constants

Gets the PID constants for the length control of the actuator. This is useful for checking the current PID constants of the actuator.
Use the `ActuatorConstants::GET_LENGTH_PID` to call this

Payload[0]

#### Returns

Payload[12]:

-   0: float32 high byte of the proportional constant
-   1: float32 mid byte of the proportional constant
-   2: float32 mid byte of the proportional constant
-   3: float32 low byte of the proportional constant
-   4: float32 high byte of the integral constant
-   5: float32 mid byte of the integral constant
-   6: float32 mid byte of the integral constant
-   7: float32 low byte of the integral constant
-   8: float32 high byte of the derivative constant
-   9: float32 mid byte of the derivative constant
-   10: float32 mid byte of the derivative constant
-   11: float32 low byte of the derivative constant
