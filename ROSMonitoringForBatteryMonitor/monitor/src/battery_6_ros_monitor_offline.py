#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
from threading import *
from rospy_message_converter import message_converter
from monitor.msg import *
from std_msgs.msg import String
from uav_msgs.msg import *


battery_status_topic = '/battery_monitor/battery_status'
input_accepted_topic = '/battery_monitor/input_accepted'


ws_lock = Lock()

def callback_battery_monitor_battery_status(data):
	global ws, ws_lock
	rospy.loginfo('monitor has observed: ' + str(data))
	dict = message_converter.convert_ros_message_to_dictionary(data)
	dict['topic'] = battery_status_topic
	dict['time'] = rospy.get_time()
	ws_lock.acquire()
	logging(dict)
	ws_lock.release()
	#rospy.loginfo('event has been successfully logged')
def callback_battery_monitor_input_accepted(data):
	global ws, ws_lock
	rospy.loginfo('monitor has observed: ' + str(data))
	dict = message_converter.convert_ros_message_to_dictionary(data)
	dict['topic'] = input_accepted_topic
	dict['time'] = rospy.get_time()
	ws_lock.acquire()
	logging(dict)
	ws_lock.release()
	#rospy.loginfo('event has been successfully logged')
pub_dict = {}
msg_dict = { battery_status_topic: "BatteryStatus",  input_accepted_topic : "InputAccepted"}
def monitor():
	global pub_error, pub_verdict
	with open(log, 'w') as log_file:
		log_file.write('')
	rospy.init_node('battery_6_ros_monitor_offline', anonymous=True)
	pub_error = rospy.Publisher(name = 'battery_6_ros_monitor_offline/monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
	pub_verdict = rospy.Publisher(name = 'battery_6_ros_monitor_offline/monitor_verdict', data_class = String, latch = True, queue_size = 1000)
	rospy.Subscriber(battery_status_topic, BatteryStatus, callback_battery_monitor_battery_status)
	rospy.Subscriber(input_accepted_topic, InputAccepted, callback_battery_monitor_input_accepted)
	rospy.loginfo('monitor started and ready')

def logging(json_dict):
	try:
		with open(log, 'a+') as log_file:
			log_file.write(json.dumps(json_dict) + '\n')
		#rospy.loginfo('event logged')
	except:
		rospy.loginfo('Unable to log the event.')

def main(argv):
	global log, actions, ws
	log = '/home/robotlab/Practice/battery_6/log_offline.txt' 
	actions = {
		battery_status_topic : ('log', 0), 
		input_accepted_topic : ('log', 0)
	}
	monitor()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
