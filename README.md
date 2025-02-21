# Firefighting UAV
This repository contains artefacts related to modelling, design, testing, and verification of a Firefighting UAV. Because this repository uses Git submodules, when checking out make sure that the submodules are initialised, for example, using the commands:
```
git clone --recurse-submodules https://github.com/VerifiabilityNode/FFUAV.git
```

## Contents
* [fire-fighting-uav](/fire-fighting-uav/): submodule with the initial version of the ROS1 implementation of the Firefighting UAV.
* [Models](/Models/): RoboChart and system testing models, including component-level and system testing artefacts. For component-level testing, the repository contains automatically generated C++ code implementing test drivers as ROS1 nodes, as well as a version of [FF_UAV_public](/FF_UAV_public/) that is instrumented for testing of the different components.
* [ROSMonitoringForBatteryMonitor](/ROSMonitoringForBatteryMonitor/): ROS Monitors for the component battery monitor.
* [mutants](/mutants/): Mutants for the battery monitor component.