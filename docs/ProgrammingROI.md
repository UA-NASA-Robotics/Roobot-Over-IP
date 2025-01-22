# Getting started with Programming ROI

This document is a guide to getting started with programming the Roobot Over IP system. It is assumed that you have a basic understanding of the system, and have the necessary hardware and software to begin.

See here for getting started with an overview of the system or tools for programming:

-   [Overview of everything ROI](ROIAll.md)
-   [ROI programming tools](../Modules/Platformio.md)

We are assuming you are making a module from scratch, and the hardware is finished. See the electrical sub-team for help with hardware.

# 1. Make a new Module in PlatformIO

We need to start of by making a new module source tree. This referes to a folder in the `Modules` directory that contains all the code or configuration for a single module. This is done by copying the `Modules/Template` folder and renaming it to the module name.
