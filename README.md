# UAV-firefighting

This repository contains an Eclipse project with software and physical models of
the Firefighting UAV case-study. The project is nested for convenience when
loading this archive in Eclipse, with the actual models contained in the folder
`FireFightingUAV-pd`. A brief description of the models, as well as the filesystem
structure follows below. Diagrams created with RoboTool have also been added to
this repository.

## RoboChart software model

The RoboChart software model is contained in files `.rct` within the sub-folder `software`
of `FireFightingUAV-pd`. Diagrams exported using RoboTool are available in `software/diagrams`.

## RoboSim physical model

The RoboSim physical model is contained in files `.pm` within the subfolder `physical`
of `FireFightingUAV-pd`. Diagrams exported using RoboTool are available in `physical/diagrams`.

## Project filesystem structure
The RoboTool project `FireFightingUAV-pd` is structured as follows:

```
  gazebo-models/ -- contains .config files suitable for loading SDF models in Gazebo
  physical/      -- RoboSim physical model files
  sdf-gen/       -- SDF files generated from RoboSim physical model with RoboTool
  software/      -- RoboChart software model
  representations.aird -- RoboTool diagrams
```
