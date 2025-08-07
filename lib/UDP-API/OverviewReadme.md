# Module Codec Lookup

Wiki for looking up the UDP codec/api for each module. This is a reference for developers to understand the codec for each module. Used with the `ModuleCodec.h` file. Quick link to different modules and packet types:

Table of Contents

-   [sysAdmin](sysAdmin.md)
-   [moduleTypes](moduleTypes.md) (UIDs for each module)
-   [GeneralGPIO](generalGPIO.md)
    -   [Module README](../../Modules/GPIO%20ROI%20Module/README.md)
-   [ODrive](oDrive.md)
    -   [Module README](../../Modules/ODrive%20ROI%20Module/README.md)
-   [Actuator Module](actuator.md)

    -   [Module README](../../Modules/Actuator%20ROI%20Module/README.md)

-   Internal
    -   [UDP Packet Structure](#communication)
    -   [Best Practices](#best-practices)

## Communication

### UDP Communication

UDP is used for most communications; most traffic follows a command and response pattern, with the controller sending a command and the module responding with a response. If no coherent response is received, the controller will resend the command.

The UDP packets are sent on different ports that support different structures, but all are variable length. General packets, used to configure, read, write, or update the module are sent on port 57344, `ROICONSTANTS::ROI_GENERAL_PORT`, and are structured as follows:

![General Packet](/docs/img/general-Packet.png)

A 16-bit subDeviceID is used to identify the module subsystem, a GPIO pin for example, and a 16-bit action code is used to identify the action to be taken on the subsystem. Finally, a data payload is included, which can be of variable length up to `ROICONSTANTS::MAXPACKETPAYLOAD`.

sysAdmin packets are sent on port 57664, `ROICONSTANTS::ROI_SYS_ADMIN_PORT`, and are used to manage the module and network. They can be used to ping, generate a status report, or configure other setting of the module. All modules will support all sysAdmin packets, whereas general packets are module specific. The structure of a sysAdmin packet is as follows:

![SysAdmin Packet](/docs/img/sysAdmin-Packet.png)

The packet has an additional 16-bit metadata field, which is used to set [chain parameters](#neighbor-chain), such as whether a packet should be forwarded to the next module in the chain, and where replies should be sent. The origin octet is used to identify the module that started the chain, and the forwarding stops when the packet reaches the origin module. The sysAdmin packets have an action code used to specify the action to be taken, and a data payload that can be used to pass additional information.

### TCP Communication

TCP is used for interrupt signals from modules, as it creates a reliable stream where interrupts can be sent without response or worry of lost packets. Currently interrupts are work-in-progress, but the plan is to have them sent on port 57600, `ROICONSTANTS::ROIINTERRUPTPORT`.

## Neighbor Chain

ROI modules are made to be connected on a network with non-ROI devices and have dynamic IP configuration(WIP), and because of this sending packets to all devices is more difficult. To solve this each module is tasked with finding it's closes ROI neighbor and forwarding packets to it when appropriate. As each module finds it's neighbor, a chain will be formed where each module forwards a packet to the next and to the next, reaching all of the devices. This chain is dynamic and will handle hot-plug of modules into and out of the network.

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
