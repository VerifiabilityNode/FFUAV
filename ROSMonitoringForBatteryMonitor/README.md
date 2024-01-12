# ROS Monitoring for the FUAV Battery Monitor
## Case Study Outline

![Untitled](https://github.com/LilithMary/FUAV-Battery-Monitor/assets/68646445/12883cb8-8591-4ce3-bfb0-3e6a8e9a9edb)


ROS nodes:
- battery:
     - subscribes to: nothing
     - publishes: topic ```"/battery_state"```
- batteryMonitor:
     - subscribes to: topic ```"/battery_state"```
     - publishes: topics ```"/input_accepted"``` and ```"/battery_status"```
- ROSMonitor:
     - subscribes to: topics ```"/battery_state"```, ```"/input_accepted"```, and ```"/battery_status"```
     - publishes: topic ```"verdict"```
 
How is the verdict determined? If the batteryMonitor node behaves *correctly*, then the verdict should be ```True```, and otherwise ```False```. The *correct* behaviour of the node was provided as a state machine in the RoboChart modelling language. The properties in the next section are formulated to characterise the intended behaviour. 

### Properties in Temporal Logic
All forbidden behaviours listed below were systematically generated from the RoboChart model. Each identified forbidden behaviour is followed by a property preventing it:

1. input accepted without an input:<br />
<!-- ```forall[i]. (acc(i) -> in(i))``` <br /> -->
     (forall[i]. {topic: "/input_accepted", id: *i} -> once({topic: "/battery_state", id: *i}))

2. status without an input accepted: <br />
<!-- ```forall[i]. (out(i) -> acc(i))```<br /> -->
     (forall[i]. {topic: "/battery_status", id: *i} -> once({topic: "/input_accepted", id: *i}))

3. incorrect status: <br />
<!-- ```forall[i]. (forall[s]. (out(i, s) -> in(i, s)))```<br /> -->
     (forall[i]. (forall[s]. {topic: "/battery_status", status: *s, id: *i} -> once({topic: "/battery_state", "percentage": *s, id: *i})))

4. multiple inputs accepted: <br />
<!-- ```acc -> forall[i]. historically[1:] (not acc(i) or once(out(i)))```<br /> -->
     (forall[i]. {topic: "/input_accepted", id: *i} -> (historically[1:](not {topic: "/input_accepted", id: *i})) and (forall[j]. (historically[1:] (not {topic: "/input_accepted", id: *j})) or (once({topic: "/battery_status", id: *j}))))


5. multiple statuses for the same input: <br />
<!-- ```forall[i]. out(i) -> historically[1:] (not out(i))```<br /> -->
     (forall[i]. ({topic: "/battery_status", id: *i} -> (historically[1:] (not {topic: "/battery_status", id: *i}))))

## Set up the workspace
1. Create a catkin workspace, make, and source it:
   ```
   source /opt/ros/noetic/setup.bash
   mkdir -p ~/<your_catkin_ws>/src
   catkin_make
   source devel/setup.bash
   ```
2. Set up the ROS packages and source the catkin workspace again:
   Download the all forders including battery_monitor, uav_msgs, and monitor folders, unzip, and place them in ```~/<your_catkin_ws>/src```. Then run the following commands:
   
   ```
   catkin_make --only-pkg-with-deps battery_monitor
   catkin_make --only-pkg-with-deps monitor
   source devel/setup.bash
   ```
   Make the monitor codes executable:
   ```
   chmod +x ~/<your_catkin_ws>/src/monitor/src/battery_ros_mon_offline.py
   chmod +x ~/<your_catkin_ws>/src/monitor/src/battery_ros_mon_online.py
   ```
4. Set up ROSMonitoring package as explained [here](https://github.com/fatmaf/ROSMonitoring/tree/master)
5. For monitoring with properties in Temporal Logic:<br />
     Download ```ros_mon_temporal_properties.py``` and move it to ```~/<your_ROSMonitoring_path>/ROSMonitoring/oracle/TLOracle```
 
## Offline monitoring:
Here we run the ROS nodes first and let the offline monitor log the events. Once the execution is finished, we use the offline oracle to verify the logged events.

#### Terminal 1: Run ROS core service
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roscore
```
#### Terminal 2: Run ROS monitoring node
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch monitor run_offline.launch
```

#### Terminal 3: Run battery and battery monitor nodes
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch battery_monitor battery_monitor.launch
```

Interrupt the runs in each terminal to end the processes. 
All events observed by the ROS monitor should be in ```~/<your_catkin_ws>/log_offline.txt```.

#### Terminal 4: Run offline TL oracle to process logged events
```
cd ~/<your_ROSMonitoring_path>/ROSMonitoring/oracle/TLOracle
./oracle.py --offline --property ros_mon_temporal_properties --trace ~/<your_catkin_ws>/log_offline.txt --discrete
```

## Online monitoring:
Here we run the online oracle before running the ROS nodes so that the verdicts are published as the nodes are active.

#### Terminal 1: Run online TL oracle on Webserver  

```
cd ~/<your_ROSMonitoring_path>/ROSMonitoring/oracle/TLOracle
./oracle.py --online --property ros_mon_temporal_properties --port 8080 --discrete
```

#### Terminal 2: Run ROS core service
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roscore
```
#### Terminal 3: Run ROS monitoring node
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch monitor run_online.launch
```

#### Terminal 4: Run battery and battery monitor nodes
```
cd ~/<your_catkin_ws>
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch battery_monitor battery_monitor.launch
```

Interrupt the runs in each terminal to end the processes. 
All events observed by the ROS monitor should be in ```~/<your_catkin_ws>/log_online.txt```.


<!--
# Battery_ROS_Monitor

## Run Offline Monitor

### First part
```
cd ~/Practice/battery_ros_mon 
```

### Repeated part
```
source /opt/ros/noetic/setup.bash 
source devel/setup.bash 
```

### Terminal 1
```roscore```

### Terminal 2
```roslaunch monitor run_offline.launch```

### Terminal 3
```roslaunch battery_monitor battery_monitor.launch```

### Terminal 4
```cd ../ROSMonitoring/oracle/TLOracle/```<br>

```
./oracle.py --offline --property ros_mon_temporal_properties --trace ~/Practice/battery_ros_mon/log_offline.txt --discrete
```

## Run Online Monitor

### Terminal 0
```cd ~/Practice/ROSMonitoring/oracle/TLOracle```<br>
```./oracle.py --online --property ros_mon_temporal_properties --port 8080 --discrete```

### Terminal 2 
```roslaunch monitor run_online.launch```

----
## Generate ROS Monitors
```
cd ~/Practice/ROSMonitoring/generator
./generator --config_file battery_ros_mon_online.yaml
```
Correct the topics and message import

```
chmod +x src/monitor/src/battery_ros_mon_offline.py
chmod +x src/monitor/src/battery_ros_mon_online.py
catkin_make --only-pkg-with-deps monitor

catkin_make --only-pkg-with-deps battery_monitor
```
-->
