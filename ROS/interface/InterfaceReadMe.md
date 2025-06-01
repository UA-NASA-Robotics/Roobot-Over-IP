# ROI Ros Interfaces

Definitions and purpose of the ROS interfaces used in the ROI project.

## Table of Contents

-   [General](#general)
-   Module Interfaces (External Application Interfaces)
    -   [General GPIO Module](GeneralGPIO.md)
    -   [O Drive Module](ODrive.md)
    -   [Actuator Module](Actuator.md)

## General

These are general purpose interfaces applicable to all modules in the ROI project.

-   [Health MSG](#health-msg)
-   [Queue Serialized General/SysAdmin Packet SRV](#queue-serialized-generalsysadmin-packet-srv)

### Health MSG

The heath message is a human readable topic reporting the operational status of a module. It is published by all modules in the ROI project.

Structure:

-   bool `module_connection` - True if the node has made a connection to the module, false otherwise.
-   bool `module_operational` - True if the module is operational, false otherwise.
-   uint8 `module_state` - The state of the module, see `ModuleCodec.h` for the possible values.
-   bool `module_error` - True if the module has reported an error, false otherwise. Note that this is not the same as the module operational.
-   string `module_error_message` - A human readable message describing the error if the module has reported an error.

### Queue Serialized General/SysAdmin Packet SRV

This service is published by the Transport Agent in the ROI system. It is used to queue a serialized general or sysadmin packet for transmission to a module. It is intended to be interfaced by module representation nodes, but may be directly accessed. It is non-blocking and returns immediately.

Structure:

-   Inputs:
    -   SerializedPacket `packet` - The serialized general or sysadmin packet to queue for transmission. (See SerializedPacket MSG)
-   Outputs:
    -   bool `success` - True if the packet was successfully queued for transmission, false otherwise.

Assume the service was successful as long as the health message does not report an error.
