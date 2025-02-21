# test-gen
This folder contains sets of automatically generated forbidden traces for state machines of the firefighter,
expressed using two different dialects, CSPM and RoboChart traces. Files with a suffix '_target' use the latter
RoboChart dialect of traces that can be used by the ROS Test generator to generate ROS code that implements
test drivers to execute the traces. Files without this suffix '_target' contain the raw output generated
by the RoboChart forbidden trace generator and adopt the conventions of CSPM.

## List of files

| Component       | RoboChart forbidden traces                                                    | CSPM forbidden traces                                                        |
------------------|-------------------------------------------------------------------------------|------------------------------------------------------------------------------|
| BatteryMonitor  | [BatteryMonitor_target.rtspec](BatteryMonitor_target.rtspec)                  | [file_monitor_BatteryMonitor.rtspec](file_monitor_BatteryMonitor.rtspec)     |
| WaterMonitor    | [WaterMonitor_target.rtspec](WaterMonitor_target.rtspec)                      | [file_monitor_WaterMonitor.rtspec](file_monitor_WaterMonitor.rtspec)         |
| FCS             | [file_ctrl_no_relative_target.rtspec](file_ctrl_no_relative_target.rtspec)    | [file_ctrl_no_relative.rtspec](file_ctrl_no_relative.rtspec)                 |

## Format conversion
Conversion between from CSPM forbidden traces to RoboChart forbidden traces can be done automatically
by running the [cspm2rc.etl](../utils/cspm2rc.etl) Epsilon ETL program available in the [utils folder](../utils/). 
Eclipse launch files for their execution against the correct `.rtpsec` files are equally available in the [utils folder](../utils/).

## Test generation
ROS Test generator accepts the RoboChart dialect as input, rather than the CSPM dialect.
