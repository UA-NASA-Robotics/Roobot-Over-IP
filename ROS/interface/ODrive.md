# O Drive Module

Interfaces for the O Drive ROI module. Note all services and actions are of the TargetJointState type, a extension of the sensor_msgs/JointState type, which includes 1 target joint state, and one output success bool, and one feedback joint state when as an action.

-   [Parameters](#parameters)
-   Messages
    -   [Motor Kinematic State MSG](#motor-values-msg)
    -   [Power MSG](#power-msg)
    -   [Fet Temperature MSG](#fet-temperature-msg)
    -   [Motor Temperature MSG](#motor-temperature-msg)
-   Services
    -   [Go To Absolute Position SRV](#go-to-absolute-position-srv)
    -   [Go To Relative Position SRV](#go-to-relative-position-srv)
    -   [Set Velocity SRV](#set-velocity-srv)
    -   [Set Torque SRV](#set-torque-srv)
-   Actions
    -   [Go to Position ACT](#go-to-position-act)
    -   [Go to Relative Position ACT](#go-to-relative-position-act)

### Parameters:

-   uint16_t `max_velocity` - The maximum velocity of the O Drive module in rad/s. Used for verifying the velocity feedforward value.
-   uint16_t `max_torque` - The maximum torque of the O Drive module in Nm. Used for verifying the torque feedforward value.

Bypassing this validation and commanding velocities or torques outside the ODrive's set point will trigger and over-velocity/current error in the ODrive itself, stopping the motor. The ROI module will attempt to auto-reset the ODrive after this error.

### Motor State MSG

The current motor values including position, velocity, and torque of the O Drive module. The units are rev, rev/s, and Nm respectively.

Topic name: `roi_ros/odrv/state`

Structure (Sensor msg JointState):

-   String `name` - The name of the motor, "odrv.axis*x*"
-   float `position` - The position of the O Drive module in rad.
-   float `velocity` - The velocity of the O Drive module in rad/s.
-   float `effort` - NA, 0.

This topic is updated as often as the maintain state loop is run. See the UDP API -> SysAdmin for the update rate of the maintain state loop.

### Power MSG

The voltage and current draw of the O Drive module. Volts and amps respectively.

Topic name: `roi_ros/odrv/power`

Structure:

-   float `voltage` - The voltage of the O Drive module supply.
-   float `current` - The current of the O Drive module.

### Motor Temperature MSG

The temperature values associated with the O Drive module. The units are degrees Celsius.

Topic name: `roi_ros/odrv/motor_temperature`

Structure (Sensor msg Temperature):

-   float `temperature` - The temperature of the O Drive module motor.
-   float `variance` - NA, 0.

### Fet Temperature MSG

The temperature values associated with the O Drive module FETs. The units are degrees Celsius.

Topic name: `roi_ros/odrv/fet_temperature`

Structure (Sensor msg Temperature):

-   float `temperature` - The temperature of the O Drive module FETs.
-   float `variance` - NA, 0.

### Go To Absolute Position SRV

The go to absolute position service is a service that commands the O Drive module to move to a specific position. It is non-blocking and returns immediately. Note, this switches the ODrive to position control mode.

Service name: `roi_ros/odrv/goto_position`

Structure:

-   Inputs:
    -   Sensor msg JointState `target_joint_state` - The joint state message containing the position, velocity feedforward, and torque feedforward.
        -   float `position` - The position to move to in rads.
        -   float `velocity` - The maximum velocity to move at in rads/s.
        -   float `effort` - The maximum torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the position, velocity, and torque feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go To Relative Position SRV

The go to relative position service is a service that commands the O Drive module to move to a specific position relative to its current position. It is non-blocking and returns immediately. Note, this switches the ODrive to position control mode.

Service name: `roi_ros/odrv/goto_relative_position`

Structure:

-   Inputs:
    -   Sensor msg JointState `target_joint_state` - The joint state message containing the position, velocity feedforward, and torque feedforward.
        -   float `position` - The position to move to relative to the current position in rads.
        -   float `velocity` - The maximum velocity to move at in rads/s.
        -   float `effort` - The maximum torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the position, velocity, and torque feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Velocity SRV

The set velocity service is a service that commands the O Drive module to move at a specific velocity. It is non-blocking and returns immediately. It sets the ODrive to velocity control mode.

Service name: `roi_ros/odrv/set_velocity`

Structure:

-   Inputs:
    -   Sensor msg JointState `target_joint_state` - The joint state message containing the velocity feedforward and torque feedforward.
        -   float `velocity` - The velocity to move at in rads/s.
        -   float `effort` - The maximum torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the velocity and torque feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Torque SRV

The set torque service is a service that commands the O Drive module to apply a specific torque. It is non-blocking and returns immediately. It sets the ODrive to torque control mode.

Service name: `roi_ros/odrv/set_torque`

Structure:

-   Inputs:
    -   Sensor msg JointState `target_joint_state` - The joint state message containing the torque feedforward.
        -   float `effort` - The torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the torque is valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go to Position ACT

The go to position action is an action that commands the O Drive module to move to a specific position. It is blocking and returns when the O Drive module has reached the desired position.

Action name: `roi_ros/odrv/goto_position`

Structure:

-   Inputs:
    -   Sensor msg JointState `target_joint_state` - The joint state message containing the position, velocity feedforward, and torque feedforward.
        -   float `position` - The position to move to in rads.
        -   float `velocity` - The maximum velocity to move at in rads/s.
        -   float `effort` - The maximum torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the position, velocity, and torque feedforward are valid and the O Drive module has reached the desired position, false otherwise.
-   Feedback:
    -   Sensor msg JointState `current_joint_state` - The current joint state of the O Drive module.
        -   float `position` - The current position of the O Drive module in rads.
        -   float `velocity` - The current velocity of the O Drive module in rads/s.
        -   float `effort` - NA, 0.

### Go to Relative Position ACT

The go to relative position action is an action that commands the O Drive module to move to a specific position relative to its current position. It is blocking and returns when the O Drive module has reached the desired position.

Action name: `roi_ros/odrv/goto_relative_position`

Structure:

-   Inputs:
    -   Sensor msg JointState `target_joint_state` - The joint state message containing the position, velocity feedforward, and torque feedforward.
        -   float `position` - The position to move to relative to the current position in rads.
        -   float `velocity` - The maximum velocity to move at in rads/s.
        -   float `effort` - The maximum torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the position, velocity, and torque feedforward are valid and the O Drive module has reached the desired position, false otherwise.
-   Feedback:
    -   Sensor msg JointState `current_joint_state` - The current joint state of the O Drive module.
        -   float `position` - The current position of the O Drive module in rads.
        -   float `velocity` - The current velocity of the O Drive module in rads/s.
        -   float `effort` - NA, 0.
