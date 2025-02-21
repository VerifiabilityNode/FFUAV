# UAV-firefighting

This repository contains an Eclipse project with software and physical models of
the Firefighting UAV case-study. The project is nested for convenience when
loading this archive in Eclipse, with the actual models contained in the folder
`FireFightingUAV-pd`. A brief description of the models, as well as the filesystem
structure follows below. Diagrams created with RoboTool have also been added to
this repository.

Reproduction of models and artefacts included in this repository can be done using 
[RoboTool release v1.1.2025022101](https://github.com/UoY-RoboStar/robotool/releases/tag/v1.1.2025022101).

Because this repository makes use of Git submodules, to checkout the full contents 
ensure that submodules are initialised and checked out recursively, for example,
using the following commands:
```
git submodule init
git checkout --recurse-submodules
```

## RoboChart software model

The RoboChart software model is contained in files `.rct` within the sub-folder `software`
of `FireFightingUAV-pd`. Diagrams exported using RoboTool are available in `software/diagrams`.

The RoboTool project also incudes [forbidden trace specifications](/FireFightingUAV-pd/test-gen/) and a [ROS1 catkin workspace](/FireFightingUAV-pd/catkin_ws/) with ROS nodes that implement [test drivers](/FireFightingUAV-pd/catkin_ws/src/ros-gen/) that exercise the trace specifications.

## RoboSim physical model

The RoboSim physical model is contained in files `.pm` within the subfolder `physical`
of `FireFightingUAV-pd`. Diagrams exported using RoboTool are available in `physical/diagrams`.

## Project filesystem structure
The RoboTool project `FireFightingUAV-pd` is structured as follows:

```
  catkin_ws/     -- Contains a ROS1 workspace
  csp-gen/       -- Contains a file with a configuration of datatypes used with the CSP semantics, as used, for example, to generate forbidden traces 
  gazebo-models/ -- Contains .config files suitable for loading SDF models in Gazebo
  physical/      -- RoboSim physical model files
  ros/           -- Configuration of test drivers 
  sdf-gen/       -- SDF files generated from RoboSim physical model with RoboTool
  software/      -- RoboChart software model
  test-gen/      -- Forbidden traces calculated from the RoboChart models
  utils/         -- ETL programs to translate between different trace specification languages
  representations.aird -- RoboTool diagrams
```
