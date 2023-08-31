# ROS Monitoring for the FUAV Battery Monitor

![ros_mon](https://github.com/LilithMary/FUAV-ROS-Monitor/assets/68646445/049604be-d9e4-481e-824f-26db0811421c)

ROS nodes:
- battery_monitor:
     - subscribes to: topic ```"/battery_state"```
     - publishes: topics ```"/input_accepted"``` and ```"/battery_status"```
- ROSMonitor:
     - subscribes to: topics ```"/input_accepted"```, and ```"/battery_status"```
     - publishes: topic ```"/verdict"```
 
How is the verdict determined? If the batteryMonitor node behaves *correctly*, then the verdict should be ```True```, and otherwise ```False```. The *correct* behaviour of the node was provided as a state machine in the RoboChart modelling language and captured by the [RML](https://rmlatdibris.github.io/) expression in ```valid-trace-patterns.rml```.   

--------------------------------------------------------------------------------
**Note**: the battery monitor ROS package is written by [Lenka Mudrich](https://github.com/mudrole1), modified slightly here to add ```percentage``` to messages of type ```input_accepted``` and increment IDs of each of ```input_accepted``` and ```battery_status``` messages manually and independently of the battery as well as one another.

## Set up the workspace
1. Create a catkin workspace, make, and source it:
   ```
   source /opt/ros/noetic/setup.bash
   mkdir -p ~/<your_catkin_ws>/src
   catkin_make
   source devel/setup.bash
   ```
2. Set up the ROS packages and source the catkin workspace again:
   Download the battery_monitor and monitor folders, unzip, and place them in the src folder:
   ```
   mv battery_monitor/ ~/<your_catkin_ws>/src
   mv monitor/ ~/<your_catkin_ws>/src
   catkin_make --only-pkg-with-deps battery_monitor
   source devel/setup.bash
   ```
3. Set up ROSMonitoring package as explained [here](https://github.com/fatmaf/ROSMonitoring/tree/master)
4. For monitoring with trace patters in RML, download ```valid-trace-patterns.pl``` and move it to ```~/<your_ROSMonitoring_path>/ROSMonitoring/oracle/RMLOracle/rml```


-----------------------------------------------------------------
## Offline monitoring:
Here we run the ROS nodes first and let the offline monitor log the events. Once the execution is finished, we use the offline oracle to verify the logged events.

### Terminal 1: Run ROS core service
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roscore
```
### Terminal 2: Run ROS monitoring node

Ensure the monitor python file is executable by using ```chmod +x```.
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch monitor offline_monitor.launch
```

### Terminal 3: Run battery monitor node
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch battery_monitor battery_monitor_instrumented.launch
```
### Terminal 4: Publish battery messages
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rostopic pub --once /fcs/battery_state sensor_msgs/BatteryState "percentage: 1.0"
```
Change ```percentage: 1.0``` to the desired percentage from 0.0 to 1.0 and publish as many messages as needed.
Alternatively, run the full simulation with the FCS node publishing battery messages.


Interrupt the runs in each terminal to end the processes. 
All events observed by the ROS monitor should be in ```~/<your_catkin_ws>/log_offline.txt```.

### Terminal 5: Run offline oracle to process logged events

```
cd ~/<your_ROSMonitoring_path>/ROSMonitoring/oracle/RMLOracle
swipl -p monitor=prolog prolog/offline_monitor.pl -- rml/valid-trace-patterns.pl ~/<your_catkin_ws>/log_offline.txt
```

## Online monitoring:
Here we run the online oracle before running the ROS nodes so that the verdicts are published as the nodes are active.

### Terminal 1: Run online oracle on Webserver  

```
cd ~/<your_ROSMonitoring_path>/ROSMonitoring/oracle/RMLOracle
swipl -p monitor=prolog prolog/online_monitor.pl -- rml/valid-trace-patterns.pl 8080
```
----------------------------------------------------
### Terminal 2: Run ROS core service
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roscore
```
### Terminal 3: Run ROS monitoring node
Ensure the monitor python file is executable by using ```chmod +x```.

```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch monitor online_monitor.launch
```
----------------------------------------------------
### Terminal 4: Run battery monitor node

```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash 

roslaunch battery_monitor battery_monitor_instrumented.launch
```
----------------------------------------------------
### Terminal 5: Publish battery messages

```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rostopic pub --once /fcs/battery_state sensor_msgs/BatteryState "percentage: 1.0"
```
Change ```percentage: 1.0``` to the desired percentage from 0.0 to 1.0 and publish as many messages as needed.
Alternatively, run the full simulation with the FCS node publishing battery messages.

Check the verdict on Terminal 1.

Interrupt the runs in each terminal to end the processes. 
All events observed by the ROS monitor should be in ```~/<your_catkin_ws>/log_online.txt```.
