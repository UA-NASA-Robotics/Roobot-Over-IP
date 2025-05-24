## moduleTypes

To identify each device on the network, each module type is assigned a unique ID, incrementally. This allows the ROS controller and module hardware to detect and ensure there are no mis-matches of module types within the system.

When adding a new module type, please select the next lowest available ID number.
Theoretical limit for this module type is 256 as the ID is sent as a single byte. This may be increased in the future, but for now, please keep it to a single byte.
