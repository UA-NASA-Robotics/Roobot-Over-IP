# ODrive REV compatible Module

Module made for interfacing with the ODrive motor controller and the feedback connector of the REV NEO, Neo 550, and Neo Vortex motors.

## REV 1

### Overview

### Connectors

The board has 5 connectors:

#### ODrive Unified GPIO Connector

Connects the ODrive to the module for the purposes of communication and adapting the encoder feedback signals to the ODrive.
Part No: S30B-PUDSS-1(LF)(SN)

#### REV NEO Motor Connector

Connects the REV NEO motor to the module for the purposes of interfacing with the motor's feedback signals.
Part No: B6B-PH-K-S(LF)(SN)

#### RJ45 Connector

Connects the module to the network switch. Allows communication and power to the module. 100BASE-T Compatible and IEEE 802.3af POE+ Compatible or better.

#### USB-C Connector

Connects the module to a computer for the purposes of programming and debugging. Note the USB-C connector will not power the module to avoid ground loops.

#### ICSP Connector

Connects the module to a programmer for the purposes of programming the module's microcontroller. Used for initial bootloader flashing. Can also be used for programming the microcontroller, but why would you do that to yourself?
2x3 2.54mm pitch header (Standard ICSP Layout)

### Ethernet

The module's main source of communication is the Ethernet port. The W5500 Ethernet controller interfaces between the ethernet signaling and the onboard microcontroller. It is rated at 100MBit/s and handles link negotiation and the TCP/IP stack.

Note the module is intended to act as a request response server on UDP ports. This allows it to handle requests from multiple clients and avoid issues caused by the Lunabotics Competition network delay (8s) that would cause TCP connections to timeout. Reliability is not a responsibility of the module, but the client software.

Typical response times from the module are around 5ms or 200 Requests per second. Our client software implementation in ROS determines a timeout to be 50ms + any known network delay. At the timeout, the client software will resend the request.

The ethernet controller is capable of roughly 8 TCP streams at once. These may be used in the future for more advanced control schemes.

IP Terminology:

-   x.x.x.x - IP Address, each x is a number between 0 and 255, an 8 bit number hence an octet.
-   x.x.x.y - The last octet is the host address, the first 3 octets are the network address.

The microcontroller is programmed to have a static network address and can read the host address from a dip switch at runtime. This allows the module to be easily reconfigured to avoid IP conflicts without requiring .

### Power Characteristics

The module is powered by Power over Ethernet + capable of supplying 25-30W at 57V. A TPS2378DDAR chip handles the detection and classification as a Powered Device (PD) to the POE switch, and from there a buck converter steps down the voltage to 5V for the module. A linear regulator then steps down the 5V to 3.3V for the ethernet controller and the USB system.

Total Rail Power:

-   Total: 57V, 30W
-   5V: 5V, 3A, 15W
-   3.3V: 3.3V, 700mA, 2.3W

Reserved Rail Power:

-   Total: 57V, 3.5W
-   5V: 3W
-   3.3V: 700mW

Idle Module Power Consumption: 1.5W

Reserved power is dedicated to the already existing circuitry onboard for proper operation under peak usage. Total - Reserved = Usable Power for future expansion. Note the idle draw is significantly lower than the peak reserve capacity.

### PCB

Our PCB is a 2x3" 4 layer board with a 1oz copper thickness. The reason for the size is to mirror the ODrive S1 Footprint.

The outer two layers are signal layers, and the inner two layers are a 5V and Ground plane. This allows for a low impedance path for the power and ground signals.

We opted for all components to be placed on the top side of the board to allow for easy assembly and rework. It also makes PCBA quotes cheaper.

#### Routing Specifications

Some traces have specific requirements to ensure proper operation of the module.

Obvious requirements are the sizing of power traces to handle the current. Calculate these if designing a new board based on the power available listed above.

Signal traces on the board have some more specific requirements.

Ethernet traces:
The total length of each MDI trace should be less than 2 inches, or 2000 mils. The traces should be lengthmatched within 20 mils for 1G transmissions and within 50 mils for 100M or 10M transmissions. The number of
vias and stubs on the MDI traces should be kept to a minimum.

RX traces must be length-matched to the other RX traces, and TX traces must be length-matched to the other
TX traces. The number of vias and stubs on the MII traces should be kept to a minimum.

Crosstalk must be avoided. No signals should cross unless properly separated by a ground layer. Additionally,
different differential pairs must have at least 30 mils of separation between the pairs.
As mentioned in the previous topics, traces should be length matched. To match the trace lengths, different
routing techniques can be used. It is recommended to apply those techniques on the same end of the
length-matched pair.

No metal should be under the magnetics{Isolation Transformer} on any layer. If metal is needed under the magnetics, it must be
separated by a ground plane at the least. Metal under the RJ45 connector with integrated magnetic is allowed.
(Ethernet PHY PCB Design Layout Checklist, Texas Instruments)

### Misc/Design Errors

Fixes:

-   The W5500 needs TX biasing resistors to function properly. These were added to the board and in the V2 schematic.
-   The RJ45 Leds were tied to the 5V rail, they should be tied to the 3.3V rail. This was fixed in V2.
-   The Rev Connector 5V was not bonded to the ODrive isolated 5V.
-   The octet selector MUX output enable pin was tied to ground, tri-stating the output. This was fixed in V2.
-   The isolation transformer was placed backwards removing the effect of the inductive chokes. This was fixed in V2.
-   The USB connector was slightly too far inset, making it difficult to plug in. This was fixed in V2.
-   The POE VSS VDD smoothing capacitor seemed to pull the rail over the capacitance spec of 120nf. This was removed, but the pads were left for future use.
-   The USB FTDI chip used was not capable of supplying a DTR signal needed to reset the microcontroller for programming. This was replaced in V2. The RST debounce circuit was also removed to comply with the DTR to RST scheme used in arduinos.

Additions:

-   Indicators for power and ethernet link status were added to the V2 board.
-   The octet selector MUX and counter FSM was changed to a parallel load shift register to reduce the components/cost. It did require an additional microcontroller pin to be utilized.
