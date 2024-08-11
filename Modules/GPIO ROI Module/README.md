# GPIO Module

Based closely on the Atmega328P powering it, this module has 8 digital pins and 8 analog GPIO pins. Each pin can be set as an input, input with pullup, or digital output.

## Reading

All pins have a read function, but operate differently depending on their type. Analog pins will return a value between 0 and 1023, while digital pins will return 0 or 1.

## Writing

All pins can act as digital outputs, and can be set to 0 or 1.

## Purpose

This module is a simple IO expander, and was developed as the baseline for all other modules. For more complex IO needs, see the [Module Codec](../../lib/ModuleCodec.h). There are specific use case modules such as a VX Lidar module, and more general purpose modules like the GPIO+ module.
