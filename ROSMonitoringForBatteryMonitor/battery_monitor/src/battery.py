#!/usr/bin/env python3

import rospy
from monitor.msg import *
from std_msgs.msg import String
from uav_msgs.msg import *
from sensor_msgs.msg import *

#rostopic pub --rate 5 /dji_sdk/battery_state sensor_msgs/BatteryState "percentage: 1.0" 
#listen on topic /dji_sdk/battery_state which uses this message type: sensor_msgs/BatteryState
#listen on topic /battery_status which uses this message type: battery_monitor/BatteryStatus


def battery():
    pub = rospy.Publisher('/fcs_interface/battery_state', BatteryPercentage, queue_size=1)
    rospy.init_node('battery', anonymous=True)
    rate = rospy.Rate(15) # hz
    percentage, decay = 100, 2
    i = 1
    while not rospy.is_shutdown():
        message = BatteryPercentage()
        message.percentage = percentage
        message.stamp = rospy.get_rostime()
        message.input_msg_id = i
        i+=1
        rospy.loginfo('percentage: '+str(round(message.percentage,2))+' time:'+str(message.stamp))
        pub.publish(message)
        percentage = percentage - decay if percentage >= decay else 0;
        rate.sleep()

if __name__ == '__main__':
    try:
        battery()
    except rospy.ROSInterruptException:
        pass
