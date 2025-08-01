# Roobot Over IP (ROI) — Quick Start & Typical Use

Roobot Over IP (ROI) is a modular, distributed control system designed by UA NASA Robotics for robust, scalable robotic systems. It enables hardware modules—such as motor drivers, actuators, and sensors—to be controlled over standard Ethernet/IP networks, integrating seamlessly with the Robot Operating System (ROS). ROI is particularly useful in complex robotics projects where adaptability, reliability, and maintainability are crucial.

---

## What Is ROI and Why Use It?

ROI replaces the traditional single, monolithic robot controller with a network of distributed modules. Each module—whether for motor control, sensors, or actuators—connects via Ethernet to a central switch/router, and is powered (typically via PoE+). These modules communicate using a UDP-based protocol and represent themselves in ROS as "digital twins" (nodes), making the hardware/software boundary essentially invisible to end-users.

### Why Distributed Control?

A distributed approach allows:
- **Flexible hardware layouts:** Place modules close to what they control (e.g., wheel drivers near wheels), reducing wiring complexity.
- **Scalability:** Add or swap modules as the robot evolves, with minimal rewiring or reprogramming.
- **Robustness:** If a module or even a ROS computer fails, the rest of the system can continue operating. Modules and ROS nodes can be restarted or replaced independently.

---

## Typical System Setup — UA NASA Robotics Example

A common deployment, such as the UA NASA Robotics team's 4-wheel front loader, highlights the strengths of ROI:

### Hardware Layout

- **Network Backbone:**  
  A compact Ethernet switch/router (often with PoE+) is mounted in the robot’s electronics bay. All modules and computers connect here.
- **Modules:**  
  - **4 ODrive Modules:** Each wheel uses an ODrive-based module for high-performance motor control.
  - **2-Actuator Module:** Handles the excavation subsystem (e.g., scoop or arm actuators).
- **Computers:**  
  - **On-board Processor:** (e.g., NVIDIA Jetson) runs local ROS nodes for real-time control and sensor processing.
  - **Remote Operator Station:** A laptop or desktop elsewhere on the network, running high-level ROS nodes (e.g., for teleop or monitoring).

All modules draw power and data from the same Ethernet lines (PoE+), simplifying cabling and providing reliable, high-bandwidth communication.

### Network and Module Discovery

- Modules auto-discover their closest ROI neighbor, forming a forwarding chain—ensuring that even with dynamic IPs or module hot-plugging, each device stays reachable.
- The network can include non-ROI devices as well, so the system remains flexible for expansion.

---

## Communication & Control Flow

1. **Initialization:**  
   Modules boot up, self-identify, and join the network chain. The robot’s main switch/router assigns IPs (DHCP or static).

2. **Digital Twin Creation:**  
   Each hardware module is paired with a ROS node—its digital twin—responsible for packet serialization, command logic, and state synchronization.

3. **Command/Telemetry Exchange:**  
   - **UDP:** Used for most commands (e.g., set wheel speed, move actuator). Controller sends a packet → module responds. Retransmits are handled at the software level.
   - **TCP:** Reserved for future use (e.g., asynchronous interrupts).
   - **SysAdmin Packets:** Used for network diagnostics (e.g., ping, status) and administrative commands.

4. **Typical Operation (Front Loader Example):**
   - The operator commands the front loader to drive or actuate the scoop via a ROS interface.
   - ROS nodes translate commands into ROI packets sent over the network.
   - Each ODrive module receives wheel commands, handles closed-loop control, and returns status.
   - The actuator module moves the excavator as commanded.
   - All telemetry (motor state, errors, etc.) is streamed back to ROS for display or higher-level decisions.

5. **Failure Handling:**  
   If a ROS computer fails, another can take over by spinning up the same ROS nodes. State information is recovered from the modules. Physical module swaps or reboots do not break the chain—operation resumes with minimal disruption.

---

## Programming & Extending ROI

- **Firmware:**  
  Each module runs lightweight C++ firmware using the ROI library. Main tasks: handle UDP packets, run control logic, and interface with hardware (e.g., ODrive or actuator).
- **ROS Integration:**  
  Nodes are written (in C++) to match each module. These nodes publish ROS topics, offer services/actions, and translate between ROS messages and ROI packets.
- **Customization:**  
  Developers can define new packet types, add module sub-systems, or extend ROS node functionality as needed. Documentation and code in `docs/`, `Modules/`, and `ROS/` directories provide examples and reference implementations.

---

## Key Features

- **Modular distributed architecture**
- **Simple network-based expansion**
- **Power-over-Ethernet support**
- **Transparent ROS integration**
- **Reliable UDP communication with retransmit logic**
- **Dynamic module discovery and hot-plug support**
- **Chain-based packet forwarding for robust communication**

---

## Resources & Further Reading

- [docs/ProgrammingROI.md](docs/ProgrammingROI.md): Developing new and extending modules/ROS nodes
- [ROS/README.md](ROS/README.md): ROS interface and virtualization
- [Modules/](Modules/): Example module firmware and hardware
- [lib/](lib/): Core ROI libraries

---

ROI is designed to make advanced robotics control systems more modular, maintainable, and resilient; ideal for research, competition, and field robotics.
Overview summary here provided by AI, verified for accuracy.
