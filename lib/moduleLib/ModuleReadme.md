# Module Library

These are files JUST for modules' use. The SBC will not use these files.

## Best Practices

1. Use a header guard in your header files, and encapsulate your code in a namespace. Keep constants in a separate namespace, and use `constexpr` rather than `const` for them. This will help prevent name collisions and make your code more readable. `Constexpr` is also more efficient than `const`, as it is evaluated at compile time rather than run time. (Like `#define` but with type and scope safety)

   - Example:

   ```cpp
   #ifndef MY_MODULE_H
   #define MY_MODULE_H

   namespace myModuleConstants {
       constexpr int MY_CONSTANT = 42; // Note: constexpr rather than const
   }

   namespace myModule {
       // Your code here
   }

   #endif
   ```

2. Make this library as multi-platform compatible as possible. Either avoid platform-specific code, or use preprocessor directives to handle it. Our platform build definitions are `__*__`
   - Example:
   ```cpp
   #if defined(__AVR__)
   // AVR specific code here
   EEPROM.write(0, 42);
   #elif defined(__ESP32__)
   // ESP32 specific code here
   EEPROM.write(0, 42);
   #else
    // Generic code here
   #error "Architecture not yet supported"
   #endif
   ```
3. Add helpful but not excessive debug messages. This will help you and others debug your code. Use `#ifdef DEBUG` to only include debug messages in debug builds. Save memory by using `F()` to keep literal strings from being loaded into RAM as an object. This is especially important on AVR platforms.
   - Note: Debug output to Serial is somewhat platform specific.
   - Example:
   ```cpp
   #ifdef DEBUG && defined(__AVR__)
   Serial.println(F("Debug message"));
   #endif
   ```
