# Getting started with Programming ROI

This document is a guide to getting started with programming the Roobot Over IP system. It is assumed that you have a basic understanding of ROI structure and ROS(humble) interfaces, and have the necessary hardware and software to begin.

See here for getting started with an overview of the system or tools for programming:

-   [Overview of everything ROI](ROIAll.md)
-   [ROI programming tools](../Modules/Platformio.md)

We are assuming you are making a module from scratch, and the hardware is finished. See the electrical sub-team for help with hardware.

# 1. Make a new Module in PlatformIO

We need to start of by making a new module source tree. This refers to a folder in the `Modules` directory that contains all the code or configuration for a single module. This is done by copying the `Modules/Template` folder and renaming it to the module name.

It is helpful to `CTRL+SHIFT+n` open a new window of vs code and open the new module folder. This will automatically open the "project" in PlatformIO allowing for easy compilation.

Take note of the general structure of the `main.cpp`, the infra object holds everything else required for a ROI module to successfully operate. You have access to a moduleStatusManager, and a general buffer of approximately 64 bits in length (See Packet.h for specific length macro ROI_MAX_PACKET_LENGTH). This buffer is helpful to reuse if you are short on memory. Note it is not empty, but when your code is running it can overwrite the buffer freely. If you want to develop your setup and looping functions now, go ahead and do so. Avoid making a generalPacketHandler() yet until we have defined the packet structures [step 2](#2-defining-communication-macrosconstants). If you choose to write code now, please make sure all state variables are r/w accessible. More on this in [step 2](#2-defining-communication-macrosconstants).

Note the infra.tick() in the loop function. This is required to keep the system running. It handles all the low level communication and packet handling, and many other features such as a connection watchdog.

DO NOT BLOCK THIS LINE FROM RUNNING. Please write your code in a way that either does not block the loop for any significant amount of time, or call infra.tick() within your blocking code. This is important to keep the system running and responsive. Failure to do so will result in a system reset as the watchdog timer will trigger. The watchdog timer expires after 1 second of blocking code. This is a hard limit, and cannot be changed.

# 2. Defining communication macros/constants

ROI uses UDP packets for communication due to a multitude of reasons and hardware limitations. Whether it's to our benefit or detriment, these packets are stream of data with no pre-defined order or purpose. ROI chooses to limit this for the sake of simplicity. You do not send freeform data, but rather create a Packet object with certain fields. Then the infra object serializes and transmits this data. The major benefit is the ROI system serializes and deserializes the data for you, so wether you are writing module firmware or ROS nodes, you are interacting with the exact same data structure object.

The exact form of this henceforth known as general packet (another specialized one exists but don't worry about that) is:

-   Subdevice ID (16 bit unsigned integer). This is used to refer to a device within the module. An example is more helpful: a module that controls 2 motors will use 0 and 1 as subdevice IDs. This is used to differentiate between the two motors.
-   Action Code (16 bit unsigned integer). This is used to refer to a specific action that the subdevice should take. An example is: a go clockwise action for the motors in the previous example. Note, when possible action codes should be limited in scope. Make control more granular, don't have a single action code that does a sequence unless it is absolutely necessary. Having an action code that makes a motor run forward for 1 second and then stop and then run backwards for 1 second is entirely overcomplicated. Move that sequencing logic to the ROS node and send three separate action codes. Note that control loops are an exception to this rule. The trip latency of ROI means that ideal control systems should be implemented only in firmware. However this still lends itself to simple action codes, just more complex in resulting action.
-   Checksum (16 bit unsigned integer). This is a simple checksum of the data. It is used to verify the data was not corrupted in transmission. This is handled behind the scenes without developer intervention. Just letting you know it exists. Depending on ROI settings, a failed check may result in a packet being dropped. It is the ROS node's responsibility to resend the packet if necessary (more on that later).
-   Data Payload (1-ROI_MAX_PAYLOAD_LENGTH(~60) bytes). This is held as an array of bytes for your reference. Some byte array to float methods are provided for your convenience when necessary (See floatCast.h) It is up to the developer what the payload structure is. The data supplements the action code such as: a goto position action code would be couple with a payload determining, in some units of your digression, to what position the motor should go to. In general keep the payload simple, and use small units. It is much less computationally intensive to send a distance in an integer value of millimeters than it is to send a float in meters.

Note the total general packet adds up to 64 bytes, the same length as the general buffer used to serialize the data...

We need to define these subdevice ids, action codes, and payload structure. Create a new .h and .md file in the `lib/UDP-API` folder for these constants and their descriptors.

Take a second to read a few lines of another modules api header file to familiarize yourself with the format.

I would recommend the following steps:

1. Add your module to the moduleTypes namespace in `moduleTypes.h`
2. Add a new namespace for your module in `*ModuleName*.h`, the one you've created.
3. Define your packet functions and structure in words in the `*ModuleName*.md` file.
4. Translate these into code in the `*ModuleName*.h` file. Note the descriptive types used within the file. These are used to make the code more readable and maintainable.

## State Variable Management

As mentioned previously, you should have all non-constant state variables be readable and writeable. These could include variables such as operation modes or PID constants. There should also be provisions within your communication macros to allow these values to be retrieved and updated. This allows for a more flexible/redundant system. These state variables will be stored in both the module firmware and the ROS node. If either of the two sides experiences a failure, or must be restarted, the other side can be used to recover the state. An example of this would be the module recovering from a power cycle, it will resume operation with the same state as the ROS node.

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

Here is a reference of the other infra features, in case you wish to use them:

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

This is a general purpose byte buffer of length ROI_MAX_PACKET_LENGTH. It is used to serialize and deserialize packets. It is not empty, but when your code is running it can overwrite the buffer freely. It saves memory to reuse this buffer/array over and over again.

## resetFunction()

Its simple as that. Returns the program counter to 0, and restarts from the top.
Can be used to reinit on hard fault conditions.

Infra will call it if SPI bus, W5500 chip, or ethernet link fails.

# 5. Implementing ROS interfaces

Just as we defined the packet structure before the generalPacketHandler, it's easier to work in the top-down approach to define the ROS interfaces before writing the node. These can be a lot more flexible. Implement what you see fit as useful topics services and actions. Note we will be extending a base class that implements some other required topics and services for UDP interfacing.

Now is also a good time to decide on units for your ROS interfaces. Unlike the ROI interface it may make more sense to move from the smallest possible units to the most helpful. Processing power is not a concern on a big computer.

Please familiarize yourself with `ROS/InterfaceReadMe.md` and document your interfaces for others to use in a simmilar manner to the UDP API.

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

Now this step may seem a little daunting, and it will take some time, but it is not as bad as it seems. The ROS node is just a glorified state machine that handles the communication between the module and the rest of the system. It is responsible for sending and receiving packets, and updating the state variables. This section discusses the implementation of the declaration of the ROS Node class (.h). Once all of the required functions have been defined, it is fairly straightforward to implement the .cpp file. You may want to check out pre-existing nodes for examples such as the ODrive node that will be referenced in this section.

[See Notes for configuring intellisense](#configuring-intellisense)

We will quickly review pre-made functions that you will get from the base class, and then discuss what functions you will need to implement.

## Pre-made functions

The base class implements some functions that you will need to use. These are:

-   `debugLog(std::string message)` - This is a simple debug logging function that will print to the console. It is recommended to use this instead of std::cout as it passes through the ROS logging system. This allows for better logging control and formatting.
-   `sendGeneralPacket(ROIPackets::Packet packet)` - This is a simple function that will send a packet to the module. Note you must specify the clientOctet as the module host address. This can be obtained from the getOctet() function.
-   `sendSysAdminPacket(ROIPackets::SysAdminPacket packet)` - This is a simple function that will send a sys admin packet to the module. Note you must specify the clientOctet as the module host address. This can be obtained from the getOctet() function. This may be used in functions you must implement. Sysadmin is a standardized for config settings. See CodecReadme.md for more details.
-   `publishHealthMessage()` - This is a simple function that will publish the health message to the ROS topic. This predefines the process for updating the health topic. Just update the \_healthData struct if needed, then call the function. The struct contains:
    -   bool `_module_operational` - This is a boolean value of whether the module is operational. This is controlled by the base module.
    -   uint16_t `_module_state` - This is a descriptor of the module state. This is controlled by the base module.
    -   bool `_module_error` - This is a boolean value of whether the module is in an error state. This is determined by the base module.
    -   std::string `_module_error_message` - This is a string value of the module error message. This can be manipulated by the base module, **but it mostly left for you to utilize**. Designed to be human readable.
    -   uint8_t `_timeAliveHours` - The number of hours the module has been alive. This is controlled by the base module.
    -   uint8_t `_timeAliveMinutes` - The number of minutes the module has been alive. This is controlled by the base module.
    -   uint8_t `_timeAliveSeconds` - The number of seconds the module has been alive. This is controlled by the base module.
    -   float `_supplyVoltage` - The voltage of the module. This is controlled by the base module.
    -   uint8_t `_mac[6]` - The mac address of the module. This is controlled by the base module.
-   `unpackVectorToArray(std::vector<uint8_t> vector, uint8_t \*array, uint16_t arraySize/unpackSize)` - This is a simple function that will unpack a vector to an array. This is useful for unpacking the serialized response packet into an array for import into a ROIPackets::Packet object. Note the array must be the same size or larger than the arraySize. This determines the number of bytes to unpack. This is used in the handleResponsePacket function.
-   `\_statusReportToHealthMessage(uint8_t statusReport)` - This is a simple function that will convert a status report to a health message. This is useful for converting the status report from the module to a human readable format. This is may be called in the publishHealthMessage function.
-   `getOctet()` - This is a simple function that will return the octet of the module. This is used to determine the host address of the module. It is reading a ros parameter. This is used in the sendGeneralPacket and sendSysAdminPacket functions.

## 1. Implementing the extended Node class

The first step is to actually make an extended class. `class \*yourModuleNameHere\*Module: public BaseModule`

We will need to declare the ROS interfaces we defined in the previous step. These are the topics services and actions. Note parameters are key value pairs, and are not declared.
Here is an example from the ODrive node:

```cpp
    // Msg publishers
    rclcpp::Publisher<roi_ros::msg::ODrivePower>::SharedPtr powerPublisher;
    rclcpp::Publisher<roi_ros::msg::ODriveState>::SharedPtr statePublisher;
    rclcpp::Publisher<roi_ros::msg::ODriveTemperature>::SharedPtr temperaturePublisher;

    // Service servers
    rclcpp::Service<roi_ros::srv::ODriveGotoPosition>::SharedPtr gotoPositionService;
    rclcpp::Service<roi_ros::srv::ODriveGotoRelativePosition>::SharedPtr
        gotoRelativePositionService;
    rclcpp::Service<roi_ros::srv::ODriveSetTorque>::SharedPtr setTorqueService;
    rclcpp::Service<roi_ros::srv::ODriveSetVelocity>::SharedPtr setVelocityService;

    // Action servers
    rclcpp_action::Server<roi_ros::action::ODriveGotoPosition>::SharedPtr gotoPositionActionServer;
    rclcpp_action::Server<roi_ros::action::ODriveGotoRelativePosition>::SharedPtr
        gotoRelativePositionActionServer;
```

Publishers and Services should be built in such a way that sub-device ID changes to not need to change upstream code. Each subdevice ID should get it's own publishers such that it can be remapped if the ID value changes. Ex, if you switch a motor on the left side of the bot to sub-device ID 1 via hardware changes, you should be able to remap it by topics rather than change upstream code such as arrray index values.

In the event a future developer wants to add a new interface or extend your module, it is helpful to place these, and all other relavant private methods in protected scope. This allows for easy extension of the class.

Once these have been created, it is a good time to implement any state variables you may need. Anything tunable by the developer or application specific, such as PID values should be stored as [parameters](#parameters). However anything that is not tuneable, such as target position or operation mode (ie position control/velocity control), should be stored as a state variable.

### Defining functions for custom interfaces

Once the interfaces have been declared, you will need to implement the functions that handle the interfaces. Topics are a notable exception, as they can be published to without the need for a callback function, however it is my personal preference when possible to define an easy to use `publishXTopic()` function. This is not necessary, but is recommended.

```cpp
void ODriveModule::publishPowerMessage() {
    // Publish the power message, voltage and current
    auto message = roi_ros::msg::ODrivePower();
    message.voltage = _busVoltage;
    message.current = _current;
    this->_power_publisher_->publish(message);
}
```

#### Services

Refreshing on services, they are a call response system (like an api/http request). The response is blocking, so they are intended to be used for simple commands that require a response/confirmation but not a lot of time.

As an example the ODrive node implements a setVelocity service, which verifies the validity of the service request and then immediately returns a response while queuing the command to be sent to the module.

Service servers are implemented as a callback function. This function is called when the service is called by a client. The function should be non-blocking and return immediately. One of these callback functions will need to be declared for each service you have defined. The structure follows:

```cpp
void gotoPositionServiceHandler(
        const roi_ros::srv::ODriveGotoPosition::Request::SharedPtr request,
        roi_ros::srv::ODriveGotoPosition::Response::SharedPtr response);
```

Note the request and response are structs to the request and response messages defined in your interface. For the gotoPositionService:

```ros
float position
float velocity_feedforward
float torque_feedforward
---
bool success
```

Note above the `---` is the separator between the request and response.

The available member variables are:

-   request->position
-   request->velocity_feedforward
-   request->torque_feedforward
-   response->success

#### Actions

Actions are a more complicated version of services. They are blocking functions, requiring a separate thread to handle the work. They have three callback functions, one for accepting a new request/goal, one for cancelling a goal, and one for starting the threaded goalExecution function. You will also need to define a goalExecution function that is running in another thread. This function may do nothing but report feedback coming from the module. The required callbacks include:

```cpp
    rclcpp_action::GoalResponse gotoPositionGoalHandler(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const roi_ros::action::ODriveGotoPosition::Goal> goal);

    void gotoPositionAcceptedHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoPosition>>
            goalHandle);

    rclcpp_action::CancelResponse gotoPositionCancelHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoPosition>>
            goalHandle);
```

The goalExecution function is not a callback, and so has this signature:

```cpp
    void gotoPositionExecuteHandler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoPosition>>
            goalHandle);
```

The request acceptor callback validates that an action can be completed, ie has valid parameters. If that is the case, it will pass the request down to the AcceptedHandler. The accepted handler will then start the goalExecution function. The cancel handler is called when a goal is cancelled. This is useful for stopping a goal that is in progress. The goalExecution function is the main function that will be running in a separate thread. It will be responsible for sending the command to the module and updating the feedback.

The functions are sufficiently full of boiler-plate, so just see the ODrive.cpp for a full example.

### Parameters

ROS parameters are a little more free form. Oddly enough, in contrast to everything else in ROS, the parameters are not predefined. They take the form of a key value store. The key is a string, and the value can be any ros supported type (int, float, bool, etc).

Note they must be declared in the constructor:

```cpp
this->declare_parameter("module_octet", 5);
```

The above is already taken care of by the base module.

They can then be accessed in the node as such:

```cpp
int octet = this->get_parameter("module_octet").as_int();
```

This is useful for storing values that are tunable by the developer such as PID values. You can assign a callback function that runs when the parameter is updated, thus allowing for parameters to affect the operation of the module in real time.

Callback function assignment:

```cpp
this->_octetParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&BaseModule::octetParameterCallback, this, std::placeholders::_1));
```

Callback Function Form:

```cpp
rcl_interfaces::msg::SetParametersResult BaseModule::octetParameterCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
        //...
        // This callback actually handles updating the response subscriptions of the node in the base module.
        // ie which octet the node is listening to through the transportAgentNode.
    }
```

Note the example is out of date. It seems ROS humble only allows you to have a single parameter callback function linked to all parameters, thus a polling timer is now used.

## Implementing functions

Now we will discuss the functions you will need to implement. These are:

-   `void maintainState() override` This function is responsible for essentially polling the physical module for its state. It should be non-blocking, as it will be called by a ros timer. It should request all of the state data from the module, as an example current position, velocity, current, and voltage from the ODrive. It should also periodically (Once every ~100 loops issue a sysAdmin status report packet). Is is recommend to use the scaffolding from a pre-existing node to develop this function.
-   `void responseCallback(const roi_ros::msg::SerializedPacket response) override` This function interprets response packets from the module. It should be able to handle all possible responses from the module, as defined in our [communication macros](#2-defining-communication-macrosconstants). An example of a handled response is to accept a packet about velocity data and update the state variables accordingly, then call the position/velocity publisher. [\*Transport Agent Behavior](#transport-agent-behavior)
-   `bool pushState() override` This function is responsible for sending the state of the module to the module. In the case the base module detects the module is new/fresh or unconfigured, but the ROS node state variables have been set, then there is a state mismatch. This function is responsible for pushing the ROS node state to the module. This should send as many packets as necessary to ensure all state variables have been effectively copied to the module. As an example this may be called when the physical module has been reset.
-   `void pullState() override` This is the inverse of the pushState. When the ROS node detects the module has non-default state variables, ie the is-configured flag is set, but the ROS node state variables are not set, then there is a state mismatch. This function is responsible for pulling the module state to the ROS node. This should send as many packets as necessary to ensure all state variables have been effectively copied to the ROS node. As an example this may be called when the jetson has failed and new ROS nodes are spun up on a different computer.
-   The constructor and destructor. The constructor is responsible for setting up the ROS interfaces, and the destructor is responsible for cleaning up the ROS interfaces. The constructor should also set up the state variables and parameters. The destructor should also clean up the state variables and parameters.
-   `int main(int argc, char *argv[])` This is pretty simple. We just need to actually create the node and run it. This is the entry point of the program. It is only defined in the cpp file. Not the header file.

```cpp
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<*YourNodeClassHere*>());
    rclcpp::shutdown();
    return 0;
}
```

Don't forget to write the msg, srv, and action functions if this has not been done!

# 7. CMakelists.txt

There's always one last thing to do. Thankfully this is pretty simple. Just add the following lines to the CMakelists.txt file in the `NODE EXECUTABLES` section.

```cmake
#*NodeName* Node
add_executable(*NodeName* src/*NodeName*.cpp src/*NodeName*.h src/base.h src/base.cpp) #start executable
target_link_libraries(*NodeName* lib) #link lib
target_link_libraries(*NodeName* ${cpp_typesupport_target}) #link internal interface

ament_target_dependencies(*NodeName* rclcpp std_msgs rcl_interfaces rclcpp_action rclcpp_components ) #amet linking
```

Then add your node to the install section:

```cmake
install(TARGETS
  oDrive
  *NodeName*
  DESTINATION lib/${PROJECT_NAME}
)
```

# 8. Testing

Great everything is written! Now you can compile and test your code in a ROS environment.

Godspeed.

# Notes

## Transport Agent Behavior

To optimize the number of existing ROS topics, the transportAgent will not create response topics for sys-Admin, general, or connection state of the module by default. You must first queue a general or sysAdmin packet as a form of attendance marking. The transport will know a module exists at the octet by your attempt to communicate to it, and thus create the appropriate topics.

If you leave the transportAgent running while debugging a particular node, note that each of these octets marked as "notable" will remain until the transportAgent is restarted. Improper allocation/use of these topics before "marking them as notable" may work if they have already be created by past instances of nodes, but will then fail at the next restart.

## Configuring Intellisense

To get cpp intellisense to work within the Ros/Humble docker, you need to add workspace settings to locate certain files. A premade `c_cpp_properties.json` has been provided in the docs folder. You can copy it to a .vscode folder within your workspace. This should allow VScode to identify ros specific cpp classes and namespaces and contribute meaningful error messages.

The file may need modified if you are not working in a docker container or not using ROS humble.
