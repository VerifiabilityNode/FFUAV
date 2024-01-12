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
from sensor_msgs.msg import *
from copy import copy

ws_lock = Lock()
dict_msgs = {}

topics = ['/fcs_interface/battery_state', 
	 '/battery_monitor/battery_status',
	 '/battery_monitor/input_accepted']

buffers = {topic:[] for topic in topics} #topic mapped to time stamp of **publication**
msgs_dict = dict() #time stamp of msg **publication** mapped to msg (data processed into a dictionary) 
data_dict = dict() #time stamp of msg **publication** mapped to raw data

def getTime(d): #given the message in form of a dictionary, calculates time in nanoseconds
	return pow(10,9) * d['stamp']['secs'] + d['stamp']['nsecs']

def interleaveBuffers(): # called by addToBuffer, sorts bufferred msgs according to time stamps and sends them to oracle in that order
	global buffers, msgs_dict, data_dict
	time_seq = sorted(msgs_dict)
	for t in time_seq:
		d = msgs_dict[t]
		data = data_dict[t]
		d['time'] = rospy.get_time()
		ws_lock.acquire()
		while d['time'] in dict_msgs:
			d['time'] += 0.01
		ws.send(json.dumps(d)) #<<<<<<<<<< message passed to oracle here
		dict_msgs[d['time']] = data
		ws_lock.release()	
	
	#reset the buffers
	max_time_stamp = max(map(eval,buffers['/battery_monitor/battery_status'])) #find last status msg
	msgs_temp = copy(msgs_dict) 
	data_temp = copy(data_dict)
	buffers_temp = copy(buffers)
	
	buffers = {top:[t for t in buffers_temp[top] if eval(t) > max_time_stamp] for top in topics}
	msgs_dict = {t:msg for t, msg in msgs_temp.items() if eval(t) > max_time_stamp}
	data_dict = {t:data for t, data in data_temp.items() if eval(t) > max_time_stamp}

def sendEarliestMsgToOracle():
	global buffers, msgs_dict, data_dict
	min_time = min(list(msgs_dict.keys()))
	d = msgs_dict[min_time]
	d['time'] = rospy.get_time()
	ws_lock.acquire()
	while d['time'] in dict_msgs:
		d['time'] += 0.01
	ws.send(json.dumps(d)) #<<<<<<<<<< message passed to oracle here
	dict_msgs[d['time']] = data_dict[min_time]
	ws_lock.release()	
	for topic in topics:
		if min_time in buffers[topic]:
			buffers[topic].remove(min_time)
			break
	msgs_dict.pop(min_time)
	data_dict.pop(min_time)
	
def addToBuffer(topic, d, data): #called by callback functions for all topics, buffers msgs and calls interleaveBuffers only when each topic has a msg in its buffer
	global buffers, msgs_dict, data_dict
	time_nsecs = getTime(d)	
	buffers[topic].append(time_nsecs) 
	msgs_dict[time_nsecs] = d
	data_dict[time_nsecs] = data
	non_empty_buffers = [top for top in topics if buffers[top] != []]
	while len(non_empty_buffers) == len(topics):		
		sendEarliestMsgToOracle() 		
		non_empty_buffers = [top for top in topics if buffers[top] != []]	
		

def callback_fcs_interface_battery_state(data):
	global ws, ws_lock
	rospy.loginfo('monitor has observed: ' + str(data))
	dict = message_converter.convert_ros_message_to_dictionary(data)
	dict['topic'] = '/fcs_interface/battery_state'
	addToBuffer(dict['topic'], dict, data)
#	dict['time'] = rospy.get_time()
#	ws_lock.acquire()
#	while dict['time'] in dict_msgs:
#		dict['time'] += 0.01
#	ws.send(json.dumps(dict)) #<<<<<<<<<< message passed to oracle here
#	dict_msgs[dict['time']] = data
#	ws_lock.release()
#	#rospy.loginfo('event propagated to oracle')
def callback_battery_monitor_battery_status(data):
	global ws, ws_lock
	rospy.loginfo('monitor has observed: ' + str(data))
	dict = message_converter.convert_ros_message_to_dictionary(data)
	dict['topic'] = '/battery_monitor/battery_status'
	addToBuffer(dict['topic'], dict, data)
#	dict['time'] = rospy.get_time()
#	ws_lock.acquire()
#	while dict['time'] in dict_msgs:
#		dict['time'] += 0.01
#	ws.send(json.dumps(dict)) #<<<<<<<<<< message passed to oracle here
#	dict_msgs[dict['time']] = data
#	ws_lock.release()
#	#rospy.loginfo('event propagated to oracle')
def callback_battery_monitor_input_accepted(data):
	global ws, ws_lock
	rospy.loginfo('monitor has observed: ' + str(data))
	dict = message_converter.convert_ros_message_to_dictionary(data)
	dict['topic'] = '/battery_monitor/input_accepted'
	addToBuffer(dict['topic'], dict, data)
#	dict['time'] = rospy.get_time()
#	ws_lock.acquire()
#	while dict['time'] in dict_msgs:
#		dict['time'] += 0.01
#	ws.send(json.dumps(dict)) #<<<<<<<<<< message passed to oracle here
#	dict_msgs[dict['time']] = data
#	ws_lock.release()
#	#rospy.loginfo('event propagated to oracle')
pub_dict = {}
msg_dict = { '/fcs_interface/battery_state' : "BatteryPercentage",  '/battery_monitor/battery_status' : "BatteryStatus",  '/battery_monitor/input_accepted' : "InputAccepted"}
def monitor():
	global pub_error, pub_verdict
	with open(log, 'w') as log_file:
		log_file.write('')
	rospy.init_node('battery_ros_mon_online', anonymous=True)
	pub_error = rospy.Publisher(name = 'battery_ros_mon_online/monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
	pub_verdict = rospy.Publisher(name = 'battery_ros_mon_online/monitor_verdict', data_class = String, latch = True, queue_size = 1000)
	rospy.Subscriber('/fcs_interface/battery_state', BatteryPercentage, callback_fcs_interface_battery_state)
	rospy.Subscriber('/battery_monitor/battery_status', BatteryStatus, callback_battery_monitor_battery_status)
	rospy.Subscriber('/battery_monitor/input_accepted', InputAccepted, callback_battery_monitor_input_accepted)
	rospy.loginfo('monitor started and ready')
def on_message(ws, message):
	global error, log, actions
	json_dict = json.loads(message)
	if json_dict['verdict'] == 'true' or json_dict['verdict'] == 'currently_true' or json_dict['verdict'] == 'unknown':
		if json_dict['verdict'] == 'true' and not pub_dict:
			rospy.loginfo('The monitor concluded the satisfaction of the property under analysis, and can be safely removed.')
			ws.close()
			exit(0)
		else:
			logging(json_dict)
			topic = json_dict['topic']
			rospy.loginfo('The event ' + message + ' is consistent and republished')
			if topic in pub_dict:
				pub_dict[topic].publish(dict_msgs[json_dict['time']])
			del dict_msgs[json_dict['time']]
	else:
		logging(json_dict)
		if (json_dict['verdict'] == 'false' and actions[json_dict['topic']][1] >= 1) or (json_dict['verdict'] == 'currently_false' and actions[json_dict['topic']][1] == 1):
			rospy.loginfo('The event ' + message + ' is inconsistent..')
			error = MonitorError()
			error.topic = json_dict['topic']
			error.time = json_dict['time']
			error.property = json_dict['spec']
			error.content = str(dict_msgs[json_dict['time']])
			pub_error.publish(error)
			if json_dict['verdict'] == 'false' and not pub_dict:
				rospy.loginfo('The monitor concluded the violation of the property under analysis, and can be safely removed.')
				ws.close()
				exit(0)
		if actions[json_dict['topic']][0] != 'filter':
			if json_dict['verdict'] == 'currently_false':
				rospy.loginfo('The event ' + message + ' is consistent ')
			topic = json_dict['topic']
			if topic in pub_dict:
				pub_dict[topic].publish(dict_msgs[json_dict['time']])
			del dict_msgs[json_dict['time']]
		error = True
	pub_verdict.publish(json_dict['verdict'])
	rospy.loginfo("*** verdict: "+str(json_dict['verdict']))

def on_error(ws, error):
	rospy.loginfo(error)

def on_close(ws):
	rospy.loginfo('### websocket closed ###')

def on_open(ws):
	rospy.loginfo('### websocket is open ###')

def logging(json_dict):
	try:
		with open(log, 'a+') as log_file:
			log_file.write(json.dumps(json_dict) + '\n')
		#rospy.loginfo('event logged')
	except:
		rospy.loginfo('Unable to log the event.')

def main(argv):
	global log, actions, ws
	log = '/home/robotlab/Practice/battery_ros_mon/log_online.txt' 
	actions = {
		'/fcs_interface/battery_state' : ('log', 1), 
		'/battery_monitor/battery_status' : ('log', 1), 
		'/battery_monitor/input_accepted' : ('log', 1)
	}
	monitor()
	websocket.enableTrace(False)
	ws = websocket.WebSocketApp(
		'ws://127.0.0.1:8080',
		on_message = on_message,
            
		on_error = on_error,
		on_close = on_close,
		on_open = on_open)
	ws.run_forever()

if __name__ == '__main__':
	main(sys.argv)
