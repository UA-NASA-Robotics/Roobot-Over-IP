# Actuator Module

External interfaces for communicating with the actuator module. This module is responsible for controlling the position and velocity of a linear actuator. Note that certain actuator modules may have multiple actuators, in this case the jointState messages will utilize the array format to support multiple actuators in a consistent order as defined by the actuator module pinout (ie actuator 0 is [0] entry in the jointState.)

In the case of wishing to affect a number of specific actuators at one, the name string should identify the actuator numbers, comma separated. "1,2" would affect actuators 1 and 2, while "1" would only affect actuator 1. Empty strings will affect all actuators in the module.

-   [Parameters](#parameters)
-   Messages
    -   [Kinematic State MSG](#state-msg)
-   Services
    -   [Go To Absolute Position SRV](#go-to-absolute-position-srv)
    -   [Go To Relative Position SRV](#go-to-relative-position-srv)
    -   [Set Velocity SRV](#set-velocity-srv)
-   Actions

    -   [Go to Position ACT](#go-to-position-act)
    -   [Go to Relative Position ACT](#go-to-relative-position-act)

### Parameters

-   uint8_t `actuator_count` - The number of actuators in the actuator module. This is used to determine how many actuators are present in the jointState message.
-   float[] `max_position` - The maximum position of each actuator in m, used for input validation.
-   float[] `min_position` - The minimum position of each actuator in m, used for input validation.
-   float[] `max_velocity` - The maximum velocity of each actuator in mm/s, used for input validation.

### State MSG

The state message is a topic that reports the state of the actuator module. The state includes the position and velocity of the actuator module in percent of total range and percent per second respectively.

Structure:

-   float `position` - The percentage of the actuator module's range that the actuator is currently at.
-   float `velocity` - The velocity of the actuator module in percent per second.

This topic is updated as often as the maintain state loop is run. See the Base.h for the sleep time of the maintain state loop.

### Go To Absolute Position SRV

The go to absolute position service is a service that commands the actuator module to move to a specific position. It is non-blocking and returns immediately.

Structure:

-   Inputs:
    -   float `position` - The position to move to in percent of total range.
    -   float `velocity_feedforward` - The maximum velocity to move at in percent per second.
-   Outputs:
    -   bool `success` - True if the position and velocity feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go To Relative Position SRV

The go to relative position service is a service that commands the actuator module to move to a specific position relative to its current position. It is non-blocking and returns immediately.

Structure:

-   Inputs:
    -   float `position` - The position to move to relative to the current position in percent of total range.
    -   float `velocity_feedforward` - The maximum velocity to move at in percent per second.
-   Outputs:
    -   bool `success` - True if the position and velocity feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Velocity SRV

The set velocity service is a service that commands the actuator module to move at a specific velocity. It is non-blocking and returns immediately.

Structure:

-   Inputs:
    -   float `velocity` - The velocity to move at in percent per second.
-   Outputs:
    -   bool `success` - True if the velocity is valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go to Position ACT

The go to position action is an action that commands the actuator module to move to a specific position. It is blocking and returns when the actuator module has reached the desired position.

Structure:

-   Inputs:
    -   float `position` - The position to move to in percent of total range.
    -   float `velocity_feedforward` - The maximum velocity to move at in percent per second.
-   Outputs:
    -   bool `success` - True if the position and velocity feedforward are valid and the actuator module has reached the desired position, false otherwise.
-   Feedback:
    -   float `position` - The current position of the actuator module in percent.
    -   bool `valid` - True if the arguments are valid, false otherwise.

### Go to Relative Position ACT

The go to relative position action is an action that commands the actuator module to move to a specific position relative to its current position. It is blocking and returns when the actuator module has reached the desired position.

Structure:

-   Inputs:
    -   float `position` - The position to move to relative to the current position in percent of total range.
    -   float `velocity_feedforward` - The maximum velocity to move at in percent per second.
-   Outputs:
    -   bool `success` - True if the position and velocity feedforward are valid and the actuator module has reached the desired position, false otherwise.
-   Feedback:
    -   float `position` - The current position of the actuator module in percent.
    -   bool `valid` - True if the arguments are valid, false otherwise.
