# General GPIO Module Interfaces (WIP, will change as general GPIO module is finished.)

Ros interfaces for affecting the GPIO pins on a module. This includes reading and writing pin states and values, as well as setting pin modes.

-   MSGs
    -   [Pin States MSG](#pin-states-msg)
    -   [Pin Values MSG](#pin-values-msg)
-   SRVs
    -   [Set Pin Output SRV](#set-pin-output-srv)
    -   [Set Pin Mode SRV](#set-pin-mode-srv)

### Pin States MSG

The pin states message is a topic that reports the state of all GPIO pins on a module. Pin states are the mode of the pin ie input or output.

Structure:

-   uint8[] states - An array of uint8 values representing the state of each pin on the module. The index of the array corresponds to the pin number on the module. The value of the array is the state of the pin, see `ModuleCodec.h` for the possible values.

Indices 0-7 are digital pins, 10-17 are analog pins. Any other indices are reserved for future use.

### Pin Values MSG

The pin values message is a topic that reports the value of all GPIO pins on a module. Pin values are the voltage level of the pin, ie. high or low. or analog value.

Structure:

-   uint16[] values - An array of uint16 values representing the value of each pin on the module. The index of the array corresponds to the pin number on the module. The value of the array is the value of the pin, 0-1 for digital pins, 0-1023 for analog pins.

Indices 0-7 are digital pins, 10-17 are analog pins. Any other indices are reserved for future use.

### Set Pin Output SRV

The set pin output service is a service that sets the output value of a pin on the module. It is non-blocking and returns immediately.

Structure:

-   Inputs:
    -   uint8 `pin` - The pin sub-device ID to set the output value of.
    -   uint8 `value` - The value to set the pin to, 0 for low, 1 for high.
-   Outputs:
    -   bool `success` - True if the pin ID and value are valid, false otherwise.

IDs 0-7 are digital pins, 10-17 are analog pins. Any other IDs are reserved for future use.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.

### Set Pin Mode SRV

The set pin mode service is a service that sets the mode of a pin on the module. It is non-blocking and returns immediately.

Structure:

-   Inputs:
    -   uint8 `pin` - The pin sub-device ID to set the mode of.
    -   uint8 `mode` - The mode to set the pin to, see `ModuleCodec.h` for the possible values.
-   Outputs:
    -   bool `success` - True if the pin ID and mode are valid, false otherwise.

IDs 0-7 are digital pins, 10-17 are analog pins. Any other IDs are reserved for future use.

Note the service is non-blocking and returns immediately confirming the validity of the request. Assume the update occurred successfully as long as the health message does not report an error.
