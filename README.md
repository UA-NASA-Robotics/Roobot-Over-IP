<div style="display:flex; justify-content:left;">
<img src="docs/img/roi-rounded.png" alt="Roobot-Over-IP" width="100" height="100" style="padding-right: 15px;"/>
<h1> RooBot Over IP </h1>
</div>

Intranet based IO control system for robotics and wider applications.
Modular hardware and closed loop reliable communication over ethernet, including a standard internet connected network.

## Documentation Table of Contents

-   Getting started
    -   [Overview & Introduction](docs/OverviewROI.md)
    -   [Integrating ROI in a Project](docs/Integration.md)
    -   [Developing for ROI](docs/ProgrammingROI.md)
-   Modules Reference
    -   [UDP API Reference](lib/UDP-API/OverviewReadme.md)
    -   [Module Infra Library](lib/moduleLib/ModuleReadme.md)
    -   [About PlatformIO](Modules/Platformio.md)
-   ROS Controller Reference
    -   [ROS Package Readme](ROS/README.md)
    -   [ROS External Interfaces Reference](ROS/interface/InterfaceReadMe.md)

## Layout

### System Layout

![Layout](/docs/img/Ethernet-Network.png)

Modules are individual units with various capabilities, they range from simple Arduino based GPIO to complex modules to handle sensors and communication.

Each is connected back to the network via ethernet or WIFI(WIP), and are POE capable.

The modules can be interacted with multiple controllers (any device on the network), but it's recommended to have a single controller, otherwise additional state management will be required.

### File Topology

```
|-- lib (ROI Common Library)
|   |-- moduleLib
|   |--- UDP-API
|   |-- ... (Other common library files)
|
|-- Modules (Module hardware and main src files)
|   |-- TemplateModule (Example module)
|   |   |-- src (Module source files)
|   |   |-- include (Module header files)
|   |   |-- platformio.ini (PlatformIO configuration file)
|   |
|   |-- common.ini (Common PlatformIO configuration)
|   |-- ... (Other modules)
|
|-- ROS (ROS package)
|   |-- action (ROS action files)
|   |-- interface (ROS external interfaces documentation)
|   |-- msg (ROS message files)
|   |-- srv (ROS service files)
|   |-- src (ROS package source files)
|   |-- CMakeLists.txt (ROS package build file)
|   |-- package.xml (ROS package metadata)
|
|-- docs (Documentation files)
|   |-- img (Images used in documentation)
|   |-- dev-tools (Development tools and setup files)
|
|-- testing (Test/debugging files for developers)
|-- README.md (This file)

```

## Project Proposal Presentation

<object data="docs/Roobot-Over-IP.pdf" type="application/pdf" width="700px" height="700px">
    <embed src="docs/Roobot-Over-IP.pdf">
        <p>This browser does not support PDFs. Please download the PDF to view it: <a href="docs/Roobot-Over-IP.pdf">Download PDF</a>.</p>
    </embed>
</object>

This is a project proposal presentation that was given to the team. It outlines the project and the goals of the project.

## Installation

This repo is intended to be built into your code as a submodule to the repository. Barring that, just clone the repository into a `Roobot-Over-IP` folder in your ros workspace. Through the power of recursive search colon build will find the ROI ros package and build it.

Following our best practice in your repository root for installing the ROI submodule:

1. `mkdir external`
2. `git submodule add https://github.com/UA-NASA-Robotics/Roobot-Over-IP external/Roobot-Over-IP`
3. `git submodule update --init --recursive`

We use `control_msgs` as ros interfaces, so you will need to call git submodule update --init --recursive in the `external/Roobot-Over-IP/ROS` folder to get the control_msgs package even if you don't use ROI as a submodule.

### Personal Setup

If you are setting up a system for developing ROI or using it, there are some additional steps to make work easy in vs-code.

1. Install the [PlatformIO](https://platformio.org/) extension for VSCode. (If working on embedded firmware)
2. If on windows, use our `docs/dev-tools/docker-compose.yml` file to set up a dev environment. This will port-forward ROI ports to your host machine, so you can use the dev environment to test modules and the ROI library. You can also use the dev environment to build the ROI library and modules. (WIP, may not work)
3. Setup cpp extension environment. Make a `.vscode` folder in the root of the repository and copy `docs/dev-tools/c_cpp_properties.json` into it. This will set up the include paths for the ROI library and modules allowing intellisense to work properly.

```

```
