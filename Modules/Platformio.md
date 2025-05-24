# PlatformIO

All firmware for modules are written in C++ and are compiled using PlatformIO. PlatformIO is a cross-platform, cross-architecture, and cross-framework build system that is designed to be easy to use and to work with a variety of IDEs. It is recommended to use Visual Studio Code with the PlatformIO extension, as it is the most feature-rich and well-supported code editor for PlatformIO.

## Platformio.ini

The `platformio.ini` file is the configuration file for PlatformIO. It is used to specify the build environment, the target board, and the libraries that are used in the project. The `platformio.ini` file for each module is located in the root directory of the module.

It defines specific [Libraries](#Libraries) and [Build Flags](#Build-Flags) for each module.

Note a common ini file is shared between all modules, and the specific module is defined in the `platformio.ini` file. This allows for easy switching between modules and ensures that all modules are built with the same settings.

Adding new environments or boards should be done in the common ini file unless there is a specific reason why your build should not be shared across all modules.
Additionally, your module project code should be reasonably platform agnostic. Use the `#ifdef` directive to include or exclude code based on the target platform. This allows for easy switching between platforms and ensures that all modules are built with the same settings.

### Libraries

PlatformIO can automatically manage local and remote libraries, making it easy for multiple developers to maintain a consistent library set. Libraries are specified in the `platformio.ini` file using the `lib_deps` directive. Libraries can be specified by name, by version, or by git URL.

AVR specific libraries are included using arduino-libraries repository, and should be specified down to the specific version to ensure compatibility.

PlatformIO will also include local libraries, and we make heavy use of that including the top level `lib` directory in the project. This is used to store common code that is shared between multiple modules and even the Controller.

#### Windows Users Warning

PlatformIO uses symlinks to move local libraries into the active project, but on windows this acts as a one time file copy. You will need to run `FullClean` to remove all local libraries and then run `build` to pull new versions of libraries. It's recommended to do these steps before flashing a module for production as the local libraries may have been updated.

Example `platformio.ini` library include:

```ini
lib_deps =
	LocalLib=symlink://../../../lib/ ; Top level lib, used for all modules and the main program
	arduino-libraries/Ethernet@^2.0.2
lib_ldf_mode = deep+
```

### Build Flags

Build flags are used to specify compiler options, such as module architecture and debugging. Build flags are specified in the `platformio.ini` file using the `build_flags` directive.

ROI Libraries are built to use `DEBUG` as a boolean flag to enable or disable debugging. This is done by defining `DEBUG` in the `platformio.ini` file.

The architecture of the module is specified manually in the `platformio.ini` file. This ensures that the correct code is generated for the target architecture. For AVR based modules, the architecture is specified as `__AVR__`.

Note many of the modules are built on a atmega328pb-au chip which has extra capabilities and hardware. This must be enabled with the `__328PB__` flag.

Example `platformio.ini` build flags:

```ini
build_flags =
	-D DEBUG=1 ; Enable debug serial output (this line can be set to 0 to disable debug output)
```
