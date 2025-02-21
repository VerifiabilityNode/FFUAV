# src for catkin workspace
This folder contains all the ROS code for the Firefighting UAV and its dependencies, included using Git submodules. To checkout this repository
correctly, ensure that the submodules are initialised, for example, by using the following commands:

```
git submodule init
git checkout --recurse-submodules
```

## Contents
* BehaviorTree.CPP: behaviour tree C++
* BehaviorTree.ROS: behaviour tree for ROS
* Onboard-SDK: DJI APIs
* Onboard-SDK-ROS: DJI APIs for ROS
* fire-fighting-uav: ROS nodes for the firefighter
* mutants: mutants of the battery and water monitor ROS nodes; mutants for the FCS are found within the fire-fighting-uav folder, eg. ﻿﻿https://github.com/RealRobotics/fire-fighting-uav/tree/bd192ecda0505758c3fd7b02f1d97014a72bd27d/fcs_interface/src instead
* ros-gen: automatically generated ROS nodes as test drivers from forbidden traces
* timed_roslaunch: used at some point to control delays in the execution of ROS nodes when launching them with rostest/roslaunch
