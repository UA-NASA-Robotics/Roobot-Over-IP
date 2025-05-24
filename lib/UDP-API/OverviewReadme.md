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
