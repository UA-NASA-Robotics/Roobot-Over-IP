# Module Virtualizations

In the ROI system each physical module has a digital twin, or virtual representation of itself in ROS. This virtualization is interacted with by the rest of the code base, and in turn passes these affects down to the physical module via the ROI packet protocol. This allows for the system to be modular and easily developed with.

In order to clarify the difference between the physical and virtual modules, the virtual modules are referred to as "virtualizations" in the code base. This is to avoid confusion between the physical and virtual modules.

## Code Principles

1. Each virtualization is a class that inherits from the `baseModule` class. This class enforces all inheriting classes to have ROI specific functions, and contains some helpful functions for use in the virtualizations.

2. All of a virtualizations ROI packet protocol options must be defined in the `ModuleCodec.h` and `CodecReadme.md` files. These help ensure consistency between the physical and virtual modules.

3. It is helpful for future developers to include and describe each ROS interface for a given virtualization in the `InterfaceReadme.md` file. This helps to clarify the purpose and function of each ROS topic and service.
