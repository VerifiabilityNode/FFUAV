# Catkin workspace
This is a ROS1 workspace targetting ROS Noetic under Ubuntu Linux LTS 20.04. To build it, the following 
dependencies listed below need to be installed.

## Contents
* results: contains mutants for the battery monitor and water monitor, and log files with the results of running
           component tests against them.
* src: source for the ROS1 packages used in this project.

## Dependencies
Dependencies not included in the repository, but which are necessary for compiling the source code
are listed below. This has been tested on Ubuntu Linux LTS 20.04 with ROS Noetic.

Software packages installable via `apt-get`:
* ros-noetic-nmea-msgs
* ros-noetic-image-transport
* ros-noetic-tf
* libsdl2-dev (required by `dji_sdk`)
* libusb-1.0-0 (required by `dji_sdk`)
* ffmpeg (required by `dji_sdk`)
* libzmq3-dev (required by `BehaviorTree.CPP`)
* python-is-python3 (optional, but recommended for running tests involving python via `rostest`)
* build-essential (g++, etc for building)

DJI Onboard SDK needs a separate installation step before using `catkin_make` on the workspace
broadly following the [official instructions](https://developer.dji.com/onboard-sdk/documentation/development-workflow/sample-setup.html):

1. `cd src/Onboard-SDK`
2. `mkdir build`
3. `cd build`
4. `cmake ..`
5. `make`
6. `sudo make install`

Currently the package `mission_controller` does not seem to compile properly due to an issue with `man_LIBRARIES`. A
workaround is to blacklist that package using `catkin_make -DCATKIN_BLACKLIST_PACKAGES="mission_controller"`.
