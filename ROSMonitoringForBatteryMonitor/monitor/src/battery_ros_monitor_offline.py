#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
from threading import *
from rospy_message_converter import message_converter
from monitor.msg import *
from std_msgs.msg import *

from uav_msgs.msg import BatteryStatus
from uav_msgs.msg import BatteryPercentage
from uav_msgs.msg import InputAccepted

ws_lock = Lock()
pub_dict = {}
srv_type_dict = dict()
topics_to_republish = []
msg_dict = { '/battery_monitor/battery_status' : "uav_msgs/BatteryStatus",  '/battery_monitor/input_accepted' : "uav_msgs/InputAccepted",  '/fcs_interface/battery_state' : "uav_msgs/BatteryPercentage"}

topics_to_reorder = ["/battery_monitor/battery_status", "/battery_monitor/input_accepted", "/fcs_interface/battery_state"]
buffers = {topic:[] for topic in topics_to_reorder} #topic mapped to time stamp of **publication**
msgs_dict = dict() #time stamp of msg **publication** mapped to msg (data processed into a dictionary) 
data_dict = dict() #time stamp of msg **publication** mapped to raw data

def getTime(d): #given the message in form of a dictionary, calculates time in nanoseconds
	return pow(10,9) * d['stamp']['secs'] + d['stamp']['nsecs']
	
def addToBuffer(topic, d, data): #called by callback functions for all topics, buffers msgs and calls logEarliestMsg only when every topic has a msg in its buffer
	global ws_lock, buffers, msgs_dict, data_dict
	time_nsecs = getTime(d)	
	buffers[topic].append(time_nsecs) 
	msgs_dict[time_nsecs] = d
	data_dict[time_nsecs] = data
	non_empty_buffers = [top for top in topics_to_reorder if buffers[top] != []]
	while len(non_empty_buffers) == len(topics_to_reorder):	
		ws_lock.acquire()		
		logEarliestMsg() 	
		ws_lock.release()		
		non_empty_buffers = [top for top in topics_to_reorder if buffers[top] != []]	


def logEarliestMsg():
	global data_dict, msgs_dict
	min_time = min(list(msgs_dict.keys()))
	d = msgs_dict[min_time]

	d['time'] = rospy.get_time()
	logging(d)
	for topic in topics_to_reorder:
		if min_time in buffers[topic]:
			buffers[topic].remove(min_time)
			break
	msgs_dict.pop(min_time)
	data_dict.pop(min_time)


def callback__battery_monitor_battery_status(data):
	d = message_converter.convert_ros_message_to_dictionary(data)
	d['topic'] = '/battery_monitor/battery_status' 
	rospy.loginfo('monitor has observed the following message on topic '+d['topic']+ ":\n" + str(data))
	addToBuffer(d['topic'], d, data)

def callback__battery_monitor_input_accepted(data):
	d = message_converter.convert_ros_message_to_dictionary(data)
	d['topic'] = '/battery_monitor/input_accepted' 
	rospy.loginfo('monitor has observed the following message on topic '+d['topic']+ ":\n" + str(data))
	addToBuffer(d['topic'], d, data)

def callback__fcs_interface_battery_state(data):
	d = message_converter.convert_ros_message_to_dictionary(data)
	d['topic'] = '/fcs_interface/battery_state' 
	rospy.loginfo('monitor has observed the following message on topic '+d['topic']+ ":\n" + str(data))
	addToBuffer(d['topic'], d, data)



def shutdownhook():
	sys.stderr.write('\nInterrupted\n')
	non_empty_buffers = [top for top in topics_to_reorder if buffers[top] != []]
	while len(non_empty_buffers) > 0:	
		ws_lock.acquire()		
		logEarliestMsg() 	
		ws_lock.release()		
		non_empty_buffers = [top for top in topics_to_reorder if buffers[top] != []]
		
def monitor():
	global pub_error, pub_verdict
	with open(log, 'w') as log_file:
		log_file.write('')
	rospy.init_node('battery_ros_monitor_offline', anonymous=True, disable_signals=True)
	rospy.on_shutdown(shutdownhook) 
	pub_error = rospy.Publisher(name = 'battery_ros_monitor_offline/monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
	pub_verdict = rospy.Publisher(name = 'battery_ros_monitor_offline/monitor_verdict', data_class = String, latch = True, queue_size = 1000)


	rospy.Subscriber('/battery_monitor/battery_status', BatteryStatus, callback__battery_monitor_battery_status)
	rospy.Subscriber('/battery_monitor/input_accepted', InputAccepted, callback__battery_monitor_input_accepted)
	rospy.Subscriber('/fcs_interface/battery_state', BatteryPercentage, callback__fcs_interface_battery_state)
	rospy.loginfo('monitor started and ready')

def on_message_topic(message):
	global error, log, actions
	json_dict = json.loads(message)
	verdict = json_dict['verdict']
	topic = json_dict['topic']
	msg = dict_msgs[json_dict['time']]

	if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':
		if verdict == 'true' and not pub_dict:
			rospy.loginfo('The monitor concluded the satisfaction of the property under analysis, and can be safely removed.')
			ws.close()
			exit(0)
		else:
			logging(json_dict)
			rospy.loginfo('The event ' + message + ' is consistent and republished')
			del json_dict['topic']
			del json_dict['time']
			ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
			if topic in pub_dict:
				pub_dict[topic].publish(ROS_message)
	else:
		logging(json_dict)
		rospy.loginfo('The event ' + message + ' is inconsistent.')
		publish_error('topic', topic, json_dict)
		if verdict == 'false' and not pub_dict:
			rospy.loginfo('The monitor concluded the violation of the property under analysis, and can be safely removed.')
			ws.close()
			exit(0)
		if actions[topic][0] != 'filter':
			del json_dict['topic']
			del json_dict['time']
			del json_dict['error']
			del json_dict['spec']
			ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
			if topic in pub_dict:
				pub_dict[topic].publish(ROS_message)
	publish_verdict(verdict)

def on_message_service_request(message):
	global error, log, actions, ws
	json_dict = json.loads(message)
	verdict = str(json_dict['verdict'])
	service = json_dict['service']
	
	if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':
		logging(json_dict)
		rospy.loginfo('The event ' + message + ' is consistent and the service', str(service), 'is called.')
		del json_dict['verdict']
		call_service(service, srv_type_dict[service], json_dict)
		del json_dict['request']
		msg = get_oracle_verdict(json_dict)
		return on_message_service_response(msg)
	else:
		logging(json_dict)
		rospy.loginfo('The request ' + message + ' is inconsistent.')
		publish_error('service', service, json_dict)
		publish_verdict(verdict)
		if actions[service][0] != 'filter':
			if 'verdict' in json_dict: del json_dict['verdict']
			call_service(service, srv_type_dict[service], json_dict)
			del json_dict['request']
			msg = get_oracle_verdict(json_dict)
			return on_message_service_response(msg)
		else:
			raise Exception('The request violates the monitor specification, so it has been filtered out.')

def on_message_service_response(message):
	global error, log, actions, ws
	json_dict = json.loads(message)
	verdict = str(json_dict['verdict'])
	service = json_dict['service']

	if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':
		logging(json_dict)
		rospy.loginfo('The response ' + message + ' is consistent, the result is returned.')
		return dict_msgs[json_dict['time']]
	else:
		logging(json_dict)
		rospy.loginfo('The response ' + message + ' is inconsistent.')
		publish_error('service', service, json_dict)
		publish_verdict(verdict)
		if actions[service][0] != 'filter':
			return dict_msgs[json_dict['time']]
		else:
			raise Exception('The request violates the monitor specification, so it has been filtered out.')

def call_service(service, msgType, json_dict):	
	global dict_msgs	
	rospy.wait_for_service(service)		
	add_two_ints = rospy.ServiceProxy(service, msgType)
	response = add_two_ints(dict_msgs[json_dict['time']])	
	rospy.loginfo('The service '+str(service)+' has been called.')
	json_dict['response'] = message_converter.convert_ros_message_to_dictionary(response)
	dict_msgs[json_dict['time']] = response


def get_oracle_verdict(json_dict):
	global ws_lock, ws
	ws_lock.acquire() #lock
		
	ws.send(json.dumps(json_dict))
	msg = ws.recv()
		
	ws_lock.release() #unlock
	return msg

def publish_error(topic_or_service, name, json_dict):
	global dict_msgs
	error = MonitorError()
	if topic_or_service == 'service':
		error.m_service = name
	else:
		error.m_topic = name
	error.m_time = json_dict['time']
	error.m_property = json_dict['spec']
	error.m_content = str(dict_msgs[json_dict['time']])
	pub_error.publish(error)
	error=True

def publish_verdict(verdict):
	verdict_msg = String()
	verdict_msg.data = verdict
	pub_verdict.publish(verdict_msg)


def logging(json_dict):
	try:
		with open(log, 'a+') as log_file:
			log_file.write(json.dumps(json_dict) + '\n')
		rospy.loginfo('event logged')
	except:
		rospy.loginfo('Unable to log the event.')

def main(argv):
	global log, actions, ws
	log = '/home/robotlab/ServiceExtension/battery_monitor_ws/log_offline.txt' 
	actions = {
		'/battery_monitor/battery_status' : ('log', 0), 
		'/battery_monitor/input_accepted' : ('log', 0), 
		'/fcs_interface/battery_state' : ('log', 0)
	}
	monitor()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
