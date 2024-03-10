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

ws_lock = Lock()
dict_msgs = {}
pub_dict = {}
srv_type_dict = dict()
msg_dict = { '/battery_monitor/battery_status' : "BatteryStatus",  '/battery_monitor/input_accepted' : "InputAccepted",  '/fcs_interface/battery_state' : "BatteryPercentage"}

topics_to_reorder = ["/battery_monitor/battery_status", "/battery_monitor/input_accepted", "/fcs_interface/battery_state"]
buffers = {topic:[] for topic in topics_to_reorder} #topic mapped to time stamp of **publication**
msgs_dict = dict() #time stamp of msg **publication** mapped to msg (data processed into a dictionary) 
data_dict = dict() #time stamp of msg **publication** mapped to raw data

def getTime(d): #given the message in form of a dictionary, calculates time in nanoseconds
	return pow(10,9) * d['stamp']['secs'] + d['stamp']['nsecs']
	
def addToBuffer(topic, d, data): #called by callback functions for all topics, buffers msgs and calls sendEarliestMsgToOracle only when every topic has a msg in its buffer
	global buffers, msgs_dict, data_dict
	time_nsecs = getTime(d)	
	buffers[topic].append(time_nsecs) 
	msgs_dict[time_nsecs] = d
	data_dict[time_nsecs] = data
	non_empty_buffers = [top for top in topics_to_reorder if buffers[top] != []]
	while len(non_empty_buffers) == len(topics_to_reorder):		
		sendEarliestMsgToOracle() 		
		non_empty_buffers = [top for top in topics_to_reorder if buffers[top] != []]	


def sendEarliestMsgToOracle():
	global ws, ws_lock
	min_time = min(list(msgs_dict.keys()))
	d = msgs_dict[min_time]

	d['time'] = rospy.get_time()
	ws_lock.acquire()
	while d['time'] in dict_msgs:
		d['time'] += 0.01
	ws.send(json.dumps(d))
	dict_msgs[d['time']] = data_dict[min_time]
	msg = ws.recv()
	ws_lock.release()

	for topic in topics_to_reorder:
		if min_time in buffers[topic]:
			buffers[topic].remove(min_time)
			break
	msgs_dict.pop(min_time)
	data_dict.pop(min_time)

	return on_message_topic(msg)


def callback__battery_monitor_battery_status(data):
	global ws, ws_lock
	rospy.loginfo('monitor has observed: ' + str(data))
	d = message_converter.convert_ros_message_to_dictionary(data)
	d['topic'] = '/battery_monitor/battery_status' 
	addToBuffer(d['topic'], d, data)
def callback__battery_monitor_input_accepted(data):
	global ws, ws_lock
	rospy.loginfo('monitor has observed: ' + str(data))
	d = message_converter.convert_ros_message_to_dictionary(data)
	d['topic'] = '/battery_monitor/input_accepted' 
	addToBuffer(d['topic'], d, data)
def callback__fcs_interface_battery_state(data):
	global ws, ws_lock
	rospy.loginfo('monitor has observed: ' + str(data))
	d = message_converter.convert_ros_message_to_dictionary(data)
	d['topic'] = '/fcs_interface/battery_state' 
	addToBuffer(d['topic'], d, data)


def monitor():
	global pub_error, pub_verdict
	with open(log, 'w') as log_file:
		log_file.write('')
	rospy.init_node('battery_ros_monitor_online', anonymous=True)
	pub_error = rospy.Publisher(name = 'battery_ros_monitor_online/monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
	pub_verdict = rospy.Publisher(name = 'battery_ros_monitor_online/monitor_verdict', data_class = String, latch = True, queue_size = 1000)


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
			if topic in pub_dict:
				pub_dict[topic].publish(msg)
			del dict_msgs[json_dict['time']]
	else:
		logging(json_dict)
		rospy.loginfo('The event ' + message + ' is inconsistent.')
		publish_error('topic', topic, json_dict)
		if verdict == 'false' and not pub_dict:
			rospy.loginfo('The monitor concluded the violation of the property under analysis, and can be safely removed.')
			ws.close()
			exit(0)
		if actions[topic][0] != 'filter':
			if topic in pub_dict:
				pub_dict[topic].publish(msg)
			del dict_msgs[json_dict['time']]
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
	log = '/home/robotlab/ServiceExtension/battery_monitor_ws/log_online.txt' 
	actions = {
		'/battery_monitor/battery_status' : ('log', 1), 
		'/battery_monitor/input_accepted' : ('log', 1), 
		'/fcs_interface/battery_state' : ('log', 1)
	}
	websocket.enableTrace(True)
	ws = websocket.WebSocket()
	ws.connect('ws://127.0.0.1:8080')
	rospy.loginfo('Websocket is open')
	monitor()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
