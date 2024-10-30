# ROI Ros Interfaces

Definitions and purpose of the ROS interfaces used in the ROI project.

## Table of Contents

- [ROI Ros Interfaces](#roi-ros-interfaces)
  - [Table of Contents](#table-of-contents)
  - [General](#general)
  - [ROS Interfaces](#ros-interfaces)
    - [General GPIO Module](#general-gpio-module)
    - [O Drive Module](#o-drive-module)
    - [Actuator Module](#actuator-module)

## General

These are general purpose interfaces applicable to all modules in the ROI project.

- [Health MSG](#health-msg)

### Health MSG

The heath message is a human readable topic reporting the operational status of a module. It is published by all modules in the ROI project.

Structure:

- bool `operational_status` - The operational status of the module. True if the module is operational, false otherwise.
- string `message` - A human readable message describing the operational status of the module, may be an error message.

### Queue Serialized General/SysAdmin Packet SRV

This service is published by the Transport Agent in the ROI system. It is used to queue a serialized general or sysadmin packet for transmission to a module. It is intended to be interfaced by module representation nodes, but may be directly accessed. It is non-blocking and returns immediately.

Structure:

- Inputs:
  - SerializedPacket `packet` - The serialized general or sysadmin packet to queue for transmission. (See SerializedPacket MSG)
- Outputs:
  - bool `success` - True if the packet was successfully queued for transmission, false otherwise.

Assume the service was successful as long as the health message does not report an error.

## General GPIO Module

- MSGs
  - [Pin States MSG](#pin-states-msg)
  - [Pin Values MSG](#pin-values-msg)
- SRVs
  - [Set Pin Output SRV](#set-pin-output-srv)
  - [Set Pin Mode SRV](#set-pin-mode-srv)

### Pin States MSG

The pin states message is a topic that reports the state of all GPIO pins on a module. Pin states are the mode of the pin ie input or output.

Structure:

- uint8[] states - An array of uint8 values representing the state of each pin on the module. The index of the array corresponds to the pin number on the module. The value of the array is the state of the pin, see `ModuleCodec.h` for the possible values.

Indices 0-7 are digital pins, 10-17 are analog pins. Any other indices are reserved for future use.

### Pin Values MSG

The pin values message is a topic that reports the value of all GPIO pins on a module. Pin values are the voltage level of the pin, ie. high or low. or analog value.

Structure:

- uint16[] values - An array of uint16 values representing the value of each pin on the module. The index of the array corresponds to the pin number on the module. The value of the array is the value of the pin, 0-1 for digital pins, 0-1023 for analog pins.

Indices 0-7 are digital pins, 10-17 are analog pins. Any other indices are reserved for future use.

### Set Pin Output SRV

The set pin output service is a service that sets the output value of a pin on the module. It is non-blocking and returns immediately.

Structure:

- Inputs:
  - uint8 `pin` - The pin sub-device ID to set the output value of.
  - uint8 `value` - The value to set the pin to, 0 for low, 1 for high.
- Outputs:
  - bool `success` - True if the pin ID and value are valid, false otherwise.

IDs 0-7 are digital pins, 10-17 are analog pins. Any other IDs are reserved for future use.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Pin Mode SRV

The set pin mode service is a service that sets the mode of a pin on the module. It is non-blocking and returns immediately.

Structure:

- Inputs:
  - uint8 `pin` - The pin sub-device ID to set the mode of.
  - uint8 `mode` - The mode to set the pin to, see `ModuleCodec.h` for the possible values.
- Outputs:
  - bool `success` - True if the pin ID and mode are valid, false otherwise.

IDs 0-7 are digital pins, 10-17 are analog pins. Any other IDs are reserved for future use.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

## O Drive Module

- Messages
  - [Position MSG](#position-msg)
  - [Velocity MSG](#velocity-msg)
  - [Torque MSG](#torque-msg)
  - [Voltage MSG](#voltage-msg)
  - [Current MSG](#current-msg)
  - [Fet Temperature MSG](#fet-temperature-msg)
  - [Motor Temperature MSG](#motor-temperature-msg)
- Services
  - [Go To Absolute Position SRV](#go-to-absolute-position-srv)
  - [Go To Relative Position SRV](#go-to-relative-position-srv)
  - [Set Velocity SRV](#set-velocity-srv)
  - [Set Torque SRV](#set-torque-srv)
- Actions
  - [Go to Position ACT](#go-to-position-act)
  - [Go to Relative Position ACT](#go-to-relative-position-act)

### Position MSG

The position message is a topic that reports the current position of the O Drive module. The position is a AngularMeasurement message, and the units are determined by the PositionUnits parameter in the O Drive module.

Structure:

- float `magnitude` - The magnitude of the position.
- uint8 `unit` - The units of the position as an index into the constant units array.
- string[] `units` - An array of possible units for the position.

This topic is updated as often as the maintain state loop is run. See the Base.h for the sleep time of the maintain state loop.

### Velocity MSG

The velocity message is a topic that reports the current velocity of the O Drive module. The velocity is a AngularMeasurement message, and the units are determined by the VelocityUnits parameter in the O Drive module.

Structure:

- float `magnitude` - The magnitude of the velocity.
- uint8 `unit` - The units of the velocity as an index into the constant units array.
- string[] `units` - An array of possible units for the velocity.

This topic is updated as often as the maintain state loop is run. See the Base.h for the sleep time of the maintain state loop.

### Torque MSG

The torque message is a topic that reports the current torque of the O Drive module. The torque is a AngularMeasurement message, and the units are determined by the TorqueUnits parameter in the O Drive module.

Structure:

- float `magnitude` - The magnitude of the torque.
- uint8 `unit` - The units of the torque as an index into the constant units array. (Always Nm)
- string[] `units` - An array of possible units for the torque.

### Voltage MSG

The voltage message is a topic that reports the current voltage of the O Drive module.

Structure:

- float `voltage` - The voltage of the O Drive module.

### Current MSG

The current message is a topic that reports the current current of the O Drive module.

Structure:

- float `current` - The current of the O Drive module.

### Fet Temperature MSG

The fet temperature message is a topic that reports the current fet temperature of the O Drive module.

Structure:

- float `temperature` - The fet temperature of the O Drive module.

### Motor Temperature MSG

The motor temperature message is a topic that reports the current motor temperature of the O Drive module.

Structure:

- float `temperature` - The motor temperature of the O Drive module.

### Go To Absolute Position SRV

The go to absolute position service is a service that commands the O Drive module to move to a specific position. It is non-blocking and returns immediately.

Structure:

- Inputs:
  - AngularMeasurement `position` - The position to move to. Acceptable units are rad, deg, or rev
  - AngularMeasurement `velocity_feedforward` - The maximum velocity to move at. Acceptable units are rad, deg, or rev. Assume it is unit per second.
  - AngularMeasurement `torque_feedforward` - The maximum torque to apply. Acceptable units are Nm.
- Outputs:
  - bool `success` - True if the position, velocity, and torque feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go To Relative Position SRV

The go to relative position service is a service that commands the O Drive module to move to a specific position relative to its current position. It is non-blocking and returns immediately.

Structure:

- Inputs:
  - AngularMeasurement `position` - The position to move to relative to the current position. Acceptable units are rad, deg, or rev
  - AngularMeasurement `velocity_feedforward` - The maximum velocity to move at. Acceptable units are rad, deg, or rev. Assume it is unit per second.
  - AngularMeasurement `torque_feedforward` - The maximum torque to apply. Acceptable units are Nm.
- Outputs:
  - bool `success` - True if the position, velocity, and torque feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Velocity SRV

The set velocity service is a service that commands the O Drive module to move at a specific velocity. It is non-blocking and returns immediately. It sets the ODrive to velocity control mode.

Structure:

- Inputs:
  - AngularMeasurement `velocity` - The velocity to move at. Acceptable units are rad, deg, or rev. Assume it is unit per second.
  - AngularMeasurement `torque_feedforward` - The maximum torque to apply. Acceptable units are Nm.
- Outputs:
  - bool `success` - True if the velocity and torque feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Torque SRV

The set torque service is a service that commands the O Drive module to apply a specific torque. It is non-blocking and returns immediately. It sets the ODrive to torque control mode.

Structure:

- Inputs:
  - AngularMeasurement `torque` - The torque to apply. Acceptable units are Nm.
- Outputs:
  - bool `success` - True if the torque is valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go to Position ACT

The go to position action is an action that commands the O Drive module to move to a specific position. It is blocking and returns when the O Drive module has reached the desired position.

Structure:

- Inputs:
  - AngularMeasurement `position` - The position to move to. Acceptable units are rad, deg, or rev
  - AngularMeasurement `velocity_feedforward` - The maximum velocity to move at. Acceptable units are rad, deg, or rev. Assume it is unit per second.
  - AngularMeasurement `torque_feedforward` - The maximum torque to apply. Acceptable units are Nm.
- Outputs:
  - bool `success` - True if the position, velocity, and torque feedforward are valid and the O Drive module has reached the desired position, false otherwise.
- Feedback:
  - AngularMeasurement `position` - The current position of the O Drive module.
  - bool `valid` - True if the arguments are valid, false otherwise.

### Go to Relative Position ACT

The go to relative position action is an action that commands the O Drive module to move to a specific position relative to its current position. It is blocking and returns when the O Drive module has reached the desired position.

Structure:

- Inputs:
  - AngularMeasurement `position` - The position to move to relative to the current position. Acceptable units are rad, deg, or rev
  - AngularMeasurement `velocity_feedforward` - The maximum velocity to move at. Acceptable units are rad, deg, or rev. Assume it is unit per second.
  - AngularMeasurement `torque_feedforward` - The maximum torque to apply. Acceptable units are Nm.
- Outputs:
  - bool `success` - True if the position, velocity, and torque feedforward are valid and the O Drive module has reached the desired position, false otherwise.
- Feedback:
  - AngularMeasurement `position` - The current position of the O Drive module.
  - bool `valid` - True if the arguments are valid, false otherwise.

## Actuator Module