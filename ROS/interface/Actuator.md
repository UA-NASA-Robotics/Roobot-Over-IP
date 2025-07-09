# Actuator Module

External interfaces for communicating with the actuator module. This module is responsible for controlling the position and velocity of a linear actuator. Note that certain actuator modules may have multiple actuators. To accomodate this, each actuator will have it's own sub-device ID set in hardware. Each sub-device ID gets it's own topics, such changes of sub-device ID will not affect upstream interfaces. There will only be 1 service and action of each type. The sub-device ID is passed in the parameters of the interfaces.

All services and actions use the target_joint_state types extended from the sensor_msgs JointState message.

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
-   float[3n] `velocity_pid` - The {Kp, Ki, Kd}\*n values for the velocity controller. (n is the number of actuators, kp0, ki0, kd0, kp1, ki1, kd1).
-   float[3n] `position_pid` - The {Kp, Ki, Kd}\*n values for the position controller.

### State MSG

The state message is a topic that reports the state of the actuator module. The state includes the position and velocity of the actuator module in percent of total range and percent per second respectively.

Topic: `roi_ros/act/axis0/state` (there will be `actuator_count` of these topics, `axis#` corresponds to the subdevice ids)

Structure:

-   Sensor msgs JointState:
    -   string `name` - The name of the actuator module, typically "axis#".
    -   float `position` - The current position of the actuator module in m.
    -   velocity `velocity` - The current velocity of the actuator module in mm/s.

This topic is updated as often as the maintain state loop is run. See the Base.h for the sleep time of the maintain state loop.

### Go To Absolute Position SRV

The go to absolute position service is a service that commands the actuator module to move to a specific position. It is non-blocking and returns immediately. This sets the actuator into position control mode.

Service: `roi_ros/act/go_to_absolute_position`

Structure:

-   Inputs:
    -   uint8 `sub_device_id` - The sub-device ID of the actuator to control.
    -   Sensor msgs JointState:
        -   float `position` - The position to move to in m.
        -   float `velocity` - The maximum velocity to move at in m/s.
-   Outputs:
    -   bool `success` - True if the position and velocity feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go To Relative Position SRV

The go to relative position service is a service that commands the actuator module to move to a specific position relative to its current position. It is non-blocking and returns immediately. This sets the actuator into position control mode.

Service: `roi_ros/act/go_to_relative_position`

Structure:

-   Inputs:
    -   uint8 `sub_device_id` - The sub-device ID of the actuator to control.
    -   Sensor msgs JointState:
        -   float `position` - The position to move to relative to the current position in m.
        -   float `velocity` - The maximum velocity to move at in m/s.
-   Outputs:
    -   bool `success` - True if the position and velocity feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Velocity SRV

The set velocity service is a service that commands the actuator module to move at a specific velocity. It is non-blocking and returns immediately. This sets the actuator into velocity control mode.

Service: `roi_ros/act/set_velocity`

Structure:

-   Inputs:
    -   uint8 `sub_device_id` - The sub-device ID of the actuator to control.
    -   float `velocity` - The velocity to move at in percent per second.
-   Outputs:
    -   bool `success` - True if the velocity is valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go to Position ACT

The go to position action is an action that commands the actuator module to move to a specific position. It is blocking and returns when the actuator module has reached the desired position. This sets the actuator into position control mode.

Action: `roi_ros/act/go_to_position`

Structure:

-   Inputs:
    -   uint8 `sub_device_id` - The sub-device ID of the actuator to control.
    -   Sensor msgs JointState:
        -   float `position` - The position to move to in m.
        -   float `velocity` - The maximum velocity to move at in m/s.
-   Outputs:
    -   bool `success` - True if the position and velocity feedforward are valid and the actuator module has reached the desired position, false otherwise.
-   Feedback:
    -   Sensor msgs JointState:
        -   float `position` - The current position of the actuator module in m.
        -   float `velocity` - The current velocity of the actuator module in m/s.

### Go to Relative Position ACT

The go to relative position action is an action that commands the actuator module to move to a specific position relative to its current position. It is blocking and returns when the actuator module has reached the desired position. This sets the actuator into position control mode.

Action: `roi_ros/act/go_to_relative_position`

Structure:

-   Inputs:
    -   uint8 `sub_device_id` - The sub-device ID of the actuator to control.
    -   Sensor msgs JointState:
        -   float `position` - The position to move to relative to the current position in m.
        -   float `velocity` - The maximum velocity to move at in m/s.
-   Outputs:
    -   bool `success` - True if the position and velocity feedforward are valid and the actuator module has reached the desired position, false otherwise.
-   Feedback:
    -   Sensor msgs JointState:
        -   float `position` - The current position of the actuator module in m.
        -   float `velocity` - The current velocity of the actuator module in m/s.
