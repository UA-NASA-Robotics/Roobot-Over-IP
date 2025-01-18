# ROI

So you want to use, understand or build the Roobot Over IP system? Better start at the beginning and work your way down this entire document to get a full understanding of the system. Good to have a surface level understanding of ROS, and UDP/TCP communication before you begin.

If just looking to reference a specific part of the system, use the links below to jump to the relevant section.

Table of Contents

-   [Distributed Controls Systems](#distributed-controls-systems)
-   [Digital Twins](#romulus-and-remus)
-   [UPD Packets](#upd-packets)
-   [Module Hardware](#module-hardware)
-   [Module Sub-Systems](#module-sub-systems)
-   [Chain Neighbors](#chain-neighbors)

# Distributed Controls Systems

ROI is a distributed control system. This means that there is no single\* central controller or PCL operating the machine.

In standard systems, you may see a monolithic controller on industrial machines, or consumer products. However, the single controller is limiting. They have set IO, abilities, and they can only be positioned in one place. This can mean as your robot changes from year to year, massive rewiring and reprogramming is needed. Your old controller might not even support everything you need anyways, then you have to shell out for an entirely new controller.

If your controller is distributed among discrete modules, you can add and remove them as needed. Plus you can place the modules where convenient. Put the motor controllers by the wheels, and the actuator controllers by the actuators. This reduces the amount of wiring needed, and makes the system more flexible.

ROI is an Ethernet/IP based distributed controls system, even a standard internet connected network. You can control the modules from anywhere on the same network subnet.

A typical layout for UA Lunabotics, is a mini-router and switch located inside the robot electrical cabinet. Then network cables are run out to modules. As a bonus, modules are POE capable, so you can run a single cable to the module sending both data and power. The current spec is POE+ or IEEE 802.11af at up to 30W.

# Romulus and Remus

The concept of a digital twin is incredibly powerful. Creating a seamless link between the physical module and a virtual representation makes the entire IP transport layer invisible to the end user. Robot Operating System, ROS, subsists of different nodes all running in isolation connected through a communications layer. ROI takes this model and applies it down to the hardware level.

ROI is actually made to interface with ROS. The digital twin representations are ROS nodes that can be interfaced over ROS topics services and actions. ROI makes this requests happen on hardware.

Using the network allows for flexibility and some sense of failure tolerance that is just not seen in microROS, given it's serial connection. If a module or even the computer running ROS fails, the system can continue to operate. The module can be replaced later, and the ROS nodes can be spun up on a new computer picking up where the others left off.

Since we have to have 2 computers on the network anyways for lunabotics, 1 Jetson for robot local processing, and 1 laptop acting a a remote operator station, we can failover ROS off the Jetson and onto the remote operator station if anything were to crash. Not the system is not truly redundant. The network is a single point of failure, but how often does a non-manageable ethernet switch fail?

Note in the rest of this document, the module refers to the physical hardware, and node refers to the virtual representation of the module in ROS.

# Module Hardware

Lets talk electrical hardware. The heart of any given module is an atmega328pb, the cooler daniel version of the Arduino Nano microcontroller.

# UPD Packets

# Module Sub-Systems

# Chain Neighbors
