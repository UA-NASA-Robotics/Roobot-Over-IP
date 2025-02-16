# Getting started with Programming ROI

This document is a guide to getting started with programming the Roobot Over IP system. It is assumed that you have a basic understanding of the system, and have the necessary hardware and software to begin.

See here for getting started with an overview of the system or tools for programming:

-   [Overview of everything ROI](ROIAll.md)
-   [ROI programming tools](../Modules/Platformio.md)

We are assuming you are making a module from scratch, and the hardware is finished. See the electrical sub-team for help with hardware.

# 1. Make a new Module in PlatformIO

We need to start of by making a new module source tree. This refers to a folder in the `Modules` directory that contains all the code or configuration for a single module. This is done by copying the `Modules/Template` folder and renaming it to the module name.

It is helpful to `CTRL+SHIFT+n` open a new window of vs code and open the new module folder. This will automatically open the "project" in PlatformIO allowing for easy compilation.

Take note of the general structure of the `main.cpp`, the infra object holds everything else required for a ROI module to successfully operate. You have access to a moduleStatusManager, and a general buffer of approximately 64 bits in length (See Packet.h for specific length macro ROIMAXPACKETLENGTH). This buffer is helpful to reuse if you are short on memory. Note it is not empty, but when your code is running it can overwrite the buffer freely. If you want to develop your setup and looping functions now, go ahead and do so. Avoid making a generalPacketHandler() yet until we have defined the packet structures [step 2](#2-defining-communication-macrosconstants). If you choose to write code now, please make sure all state variables are r/w accessible. More on this in [step 2](#2-defining-communication-macrosconstants).

Note the infra.tick() in the loop function. This is required to keep the system running. It handles all the low level communication and packet handling, and many other features such as a connection watchdog.

DO NOT BLOCK THIS LINE FROM RUNNING. Please write your code in a way that either does not block the loop for any significant amount of time, or call infra.tick() within your blocking code. This is important to keep the system running and responsive. Failure to do so will result in a system reset as the watchdog timer will trigger. The watchdog timer expires after 1 second of blocking code. This is a hard limit, and cannot be changed.

# 2. Defining communication macros/constants

ROI uses UDP packets for communication due to a multitude of reasons and hardware limitations. Whether it's to our benefit or detriment, these packets are stream of data with no pre-defined order or purpose. ROI chooses to limit this for the sake of simplicity. You do not send freeform data, but rather create a Packet object with certain fields. Then the infra object serializes and transmits this data. The major benefit is the ROI system serializes and deserializes the data for you, so wether you are writing module firmware or ROS nodes, you are interacting with the exact same data structure object.

The exact form of this henceforth known as general packet (another specialized one exists but don't worry about that) is:

-   Subdevice ID (16 bit unsigned integer). This is used to refer to a device within the module. An example is more helpful: a module that controls 2 motors will use 0 and 1 as subdevice IDs. This is used to differentiate between the two motors.
-   Action Code (16 bit unsigned integer). This is used to refer to a specific action that the subdevice should take. An example is: a go clockwise action for the motors in the previous example. Note, when possible action codes should be limited in scope. Make control more granular, don't have a single action code that does a sequence unless it is absolutely necessary. Having an action code that makes a motor run forward for 1 second and then stop and then run backwards for 1 second is entirely overcomplicated. Move that sequencing logic to the ROS node and send three separate action codes. Note that control loops are an exception to this rule. The trip latency of ROI means that ideal control systems should be implemented only in firmware. However this still lends itself to simple action codes, just more complex in resulting action.
-   Checksum (16 bit unsigned integer). This is a simple checksum of the data. It is used to verify the data was not corrupted in transmission. This is handled behind the scenes without developer intervention. Just letting you know it exists. Depending on ROI settings, a failed check may result in a packet being dropped. It is the ROS node's responsibility to resend the packet if necessary (more on that later).
-   Data Payload (1-ROIMAXPAYLOADLENTH(~60) bytes). This is held as an array of bytes for your reference. Some byte array to float methods are provided for your convenience when necessary (See floatCast.h) It is up to the developer what the payload structure is. The data supplements the action code such as: a goto position action code would be couple with a payload determining, in some units of your digression, to what position the motor should go to. In general keep the payload simple, and use small units. It is much less computationally intensive to send a distance in an integer value of millimeters than it is to send a float in meters.

Note the total general packet adds up to 64 bytes, the same length as the general buffer used to serialize the data...

We need to define these subdevice ids, action codes, and payload structure. Open the `ModuleCodec.h` and `CodecReadme.md` both located in the lib folder.

Take a second to read a few lines of each to familiarize yourself with the format.

I would recommend the following steps:

1. Add your module to the moduleTypes namespace in `ModuleCodec.h`
2. Add a new namespace for your module in `ModuleCodec.h`, preferably at the bottom of the file.
3. Define your packet functions and structure in words in the `CodecReadme.md` file.
4. Translate these into code in the `ModuleCodec.h` file. Note the descriptive types used within the file. These are used to make the code more readable and maintainable. They are not necessary, but are recommended.

## State Variable Management

As mentioned previously, you should have all non-constant state variables be readable and writeable. These could include variables such as operation modes or PID constants. There should also be provisions within your communication macros to allow these values to be retrieved and updated. This allows for a more flexible/redundant system. These state varibles will be stored in both the module firmware and the ROS node. If either of the two sides experiences a failure, or must be restarted, the other side can be used to recover the state. An example of this would be the module recovering from a power cycle, it will resume operation with the same state as the ROS node.

Additonaly, this allows some parameters to be stored as ROS parameters. For functions like PID tuning, this allows the system to update the PID constants without needing to recompile the firmware. This is especially helpful when hardware needs to be replaced, there is no need to recompile application-specific data.

The ROS node will handle the coordination of the state variables. The module firmware only needs to respond to requests for the state variables, making this step fairly simple. Just make sure to add the appropriate action codes so that state variables can be retrieved and updated.

# 3. Implementing the generalPacketHandler

Thankfully this step is some real programming. The only trickly bit is you must define your function before the construction of the infra object. Best practice would probably be using a separate file and including it. Otherwise, you can use some forward declarations or pointer tomfoolery.

Requirements for the generalPacketHandler:

1. be a function of return type ROIPackets::Packet (a reply packet)
2. accept 1 argument of type ROIPackets::Packet (the incoming packet)
3. fill out the reply packet with the appropriate data. The ROS node is expecting a reply packet containing the same subdevice and action code as that is how it ensures the command made a round trip. Best practice is to call the `.swapReply()` method on the incoming packet to create a reply packet with the correct subdevice and action code. This is not necessary, but is recommended.
4. call `infra.moduleStatusManager.notifySystemConfigured()` if any changes to the system state are made. This is important for the ROS node to know when a module is freshly initialized or has had a change in state.

The generalPacketHandler is called by the infra object when a packet is received. It is the developer's responsibility to handle the packet and return a reply packet. The infra object will handle the rest. Keep in mind you should keep in strict adherence to the packet structure defined in the previous step.

# 4. Extra Infra Features

## The moduleStatusManager

This keeps track of the modules status for reporting as well as hosts a connection watchdog.

Available functions:

-   getSystemStatus() - returns the current system status as a uint8_t. Refers to the systemStatus namespace in ModuleCodec.h
-   getOperable() - returns a boolean value of whether the module is operable. This is determined by you in other functions
-   notifyInitializedStatus() - sets the system status to initialized. This is called by you in void setup() when the module is ready to operate.
-   notifySystemConfigured() - sets the system status to configured. This is called by you in handle general packet once any changes to any state is made. _IMPORTANT_ for the ROS node to coordinate state variables.
-   notifySystemError(bool inoperable) - sets the system status to error. This is called by you when the module is in an error state. The inoperable parameter is a boolean value of whether the module is inoperable. This is used to determine if the module should be considered inoperable or just in an soft error state.
-   notifyClearError() - removes the in error flag from the module. This is called by you when the module has recovered from an error state.
-   setDisconnectCallback(void (\*callback)()) - sets a callback function to be called when the module disconnects from its twin ros node. This is useful for a pseudo E-stop on failure function.
-   setReconnectCallback(void (\*callback)()) - sets a callback function to be called when the module reconnects to its twin ros node. This is useful for a pseudo E-stop on failure function.
-   Others - don't mess with these. They are for other infra internals.

## The general buffer

This is a general purpose byte buffer of length ROIMAXPACKETLENGTH. It is used to serialize and deserialize packets. It is not empty, but when your code is running it can overwrite the buffer freely. It saves memory to reuse this buffer/array over and over again.

## resetFunction()

Its simple as that. Returns the program counter to 0, and restarts from the top.
I never remember the syntax for this, so I always have to look it up. It's `resetFunction()`.

Reserve this as a last resort. Infra will call it if SPI or the W5500 chip fails.

# 5. Implementing ROS interfaces

Just as we defined the packet structure before the generalPacketHandler, it's easier to work in the top-down approach to define the ROS interfaces before writing the node. These can be a lot more flexible. Implement what you see fit as useful topics services and actions. Note we will be extending a base class that implements some other required topics and services for UDP interfacing.

Now is also a good time to decide on units for your ROS interfaces. Unlike the ROI interface it may make more sense to move from the smallest possible units to the most helpful. Processing power is not a concern on a big computer.

Please familiarize yourself with `ROS/InterfaceReadMe.md` and document your interfaces for others to use.

Here is an example of interfaces for the ODrive Node: (Note not all are included below. See full file for all)

## O Drive Module

-   Messages
    -   Motor Kinematic State MSG
    -   Power MSG
    -   Temperature MSG
-   Services
    -   Go To Absolute Position SRV
    -   Go To Relative Position SRV
    -   Set Velocity SRV
    -   Set Torque SRV
-   Actions
    -   Go to Position ACT
    -   Go to Relative Position ACT

### Motor State MSG

The current motor values including position, velocity, and torque of the O Drive module. The units are rev, rev/s, and Nm respectively.

Topic name: `state`

Structure:

-   float `position` - The position of the O Drive module.
-   float `velocity` - The velocity of the O Drive module.
-   float `torque` - The torque of the O Drive module. (WIP. Not currently implemented)

This topic is updated as often as the maintain state loop is run. See the Base.h for the sleep time of the maintain state loop.

### Power MSG

The voltage and current draw of the O Drive module. Volts and amps respectively.

Topic name: `power`

Structure:

-   float `voltage` - The voltage of the O Drive module supply.
-   float `current` - The current of the O Drive module.

### Set Velocity SRV

The set velocity service is a service that commands the O Drive module to move at a specific velocity. It is non-blocking and returns immediately. It sets the ODrive to velocity control mode.

Service name: `set_velocity`

Structure:

-   Inputs:
    -   float `velocity` - The velocity to move at in revs/s.
    -   float `torque_feedforward` - The maximum torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the velocity and torque feedforward are valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Torque SRV

The set torque service is a service that commands the O Drive module to apply a specific torque. It is non-blocking and returns immediately. It sets the ODrive to torque control mode.

Service name: `set_torque`

Structure:

-   Inputs:
    -   float `torque` - The torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the torque is valid, false otherwise.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Go to Position ACT

The go to position action is an action that commands the O Drive module to move to a specific position. It is blocking and returns when the O Drive module has reached the desired position.

Action name: `goto_position`

Structure:

-   Inputs:
    -   float `position` - The position to move to in revs.
    -   float `velocity_feedforward` - The maximum velocity to move at in revs/s.
    -   float `torque_feedforward` - The maximum torque to apply in Nm.
-   Outputs:
    -   bool `success` - True if the position, velocity, and torque feedforward are valid and the O Drive module has reached the desired position, false otherwise.
-   Feedback:
    -   float `position` - The current position of the O Drive module.
    -   float `velocity` - The current velocity of the O Drive module.

# 6. Implementing the ROS node
