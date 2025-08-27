# Front Loader ROI Diagrams (Mermaid)

These diagrams render directly on GitHub and in VS Code (with a Mermaid extension). They mirror the draw.io content but with cleaner layout and diff-friendly text.

## Hardware overview

```mermaid
flowchart LR
  %% Layout groups
  subgraph Power[Power System]
    Battery["4s5p Li‑ion Battery"]
    BMS["BMS"]
    MainCont["Main Disconnect Contactor"]
    Dist["Distribution Block"]
    Battery --> BMS --> MainCont --> Dist
    Dist --> BrkFL["Breaker (FL)"]
    Dist --> BrkFR["Breaker (FR)"]
    Dist --> BrkRL["Breaker (RL)"]
    Dist --> BrkRR["Breaker (RR)"]
    Dist --> BrkACT["Breaker (Actuators)"]
  end

  subgraph Network[Network]
    DCPOE["DC/DC → PoE+ Injector"]
    Switch["Ethernet Switch (PoE+)"]
    Onboard["Onboard Computer (ROS 2)"]
    Operator["Operator Station (ROS 2)"]
    DCPOE --> Switch
    Operator <-->|Ethernet| Switch
    Onboard  <-->|Ethernet| Switch
  end

  subgraph Modules[ROI Modules]
    ODRV_FL["ODrive ROI Module — FL"]
    ODRV_FR["ODrive ROI Module — FR"]
    ODRV_RL["ODrive ROI Module — RL"]
    ODRV_RR["ODrive ROI Module — RR"]
    ACT["Actuator ROI Module\nPitch (ID 0) • Tilt (ID 1)"]
  end

  %% Motor power
  BrkFL -->|Motor Power| ODRV_FL
  BrkFR -->|Motor Power| ODRV_FR
  BrkRL -->|Motor Power| ODRV_RL
  BrkRR -->|Motor Power| ODRV_RR
  BrkACT -->|Actuator Power| ACT

  %% Ethernet homeruns
  Switch --> ODRV_FL
  Switch --> ODRV_FR
  Switch --> ODRV_RL
  Switch --> ODRV_RR
  Switch --> ACT

  NoteIPs["IPs may be 5–240 as needed.\nAll Ethernet runs are home-run to the switch."]
  NoteIPs --- Switch

  classDef power fill:#d5e8d4,stroke:#82b366,color:#000;
  classDef net fill:#dae8fc,stroke:#6c8ebf,color:#000;
  classDef mods fill:#fff2cc,stroke:#d6b656,color:#000;
  class Battery,BMS,MainCont,Dist,BrkFL,BrkFR,BrkRL,BrkRR,BrkACT power;
  class DCPOE,Switch,Onboard,Operator net;
  class ODRV_FL,ODRV_FR,ODRV_RL,ODRV_RR,ACT mods;
```

## Data flow

```mermaid
graph LR
subgraph Power
direction LR
%% Power Distribution
PWR[Battery]
Relay[Relay]
estop[Estop Button]
meter[Power Meter]
PWR --> estop --> Relay
PWR --> meter --> Relay

    %% Voltage rails
    Relay --> PD[Power Distribution Blocks]

    %% Actuators
    PD --> ActModule1[Act Module]
    ActModule1 --> Actuator1[Actuator]
    PD --> ActModule2[Act Module]
    ActModule2 --> Actuator2[Actuator]

    %% Drive Modules
    PD --> Drv1[Drv Module]
    Drv1 --> Wheel1

    PD --> Drv2[Drv Module]
    Drv2 --> Wheel2

    PD --> Drv3[Drv Module]
    Drv3 --> Wheel3

    PD --> Drv4[Drv Module]
    Drv4 --> Wheel4

    PD --> Boost[Boost Converter]
    PD --> Buck[DC-DC Buck Converter]

    ActLm1[Retract Limit Switch]
    ActLm2[Extend Limit Switch]
    ActLm3[Retract Limit Switch]
    ActLm4[Extend Limit Switch]

    ActLm1 --> ActModule1
    ActLm2 --> ActModule1
    ActLm3 --> ActModule2
    ActLm4 --> ActModule2

end
    Boost --> Switch
    PD --> Nano
    Buck --> Router
    %% Networking
    subgraph Comms
    Nano[Jetson Nano]
    Switch[Ethernet Switch]
    Router[Router]

    Switch --- Router
    Switch --- Nano
    Switch --- Drv1
    Switch --- Drv2
    Switch --- Drv3
    Switch --- Drv4
    Switch --- ActModule1
    Switch --- ActModule2

    %% CAM USB
    Nano --- Cam1
    Nano --- Cam2
    end
```
