#!/usr/bin/env python3

# MIT License
#
# Copyright (c) [2020] [Angelo Ferrando]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import reelay
import sys
import json
from websocket_server import WebsocketServer
from threading import *
import argparse
import importlib
from enum import Enum

# type of properties available in reelay
class TypeOfProperty(Enum):
	PLTL = 0
	PMTL = 1
	PSTL = 2

# ws_lock = Lock()

# Called for every client connecting (after handshake)
def new_client(client, server):
	print("New ROS monitor connected and was given id %d" % client['id'])
	# server.send_message_to_all("Hey all, a new client has joined us")


# Called for every client disconnecting
def client_left(client, server):
	print("ROS monitor (%d) disconnected" % client['id'])


# Called when a client sends a message
def message_received(client, server, message):
	message_dict = json.loads(message)
	if check_event(message):
		message_dict['verdict'] = 'currently_true'
	else:
		message_dict['verdict'] = 'currently_false'
		message_dict['spec'] = property.PROPERTY
	server.send_message(client, json.dumps(message_dict))

# Function checking the event against the specification (it simply calls Reelay, nothing more)
def check_event(event):
	global last_time, tl_oracle, property, last_res
	event_dict = json.loads(event)
	if 't' in event_dict and 'time' not in event_dict:
		event_dict['time'] = event_dict['t']
	if not tl_oracle:
		if model == 'dense' and 'time' in event_dict:
			tl_oracle = reelay.dense_timed_monitor(pattern = property.PROPERTY)
		else:
			tl_oracle = reelay.discrete_timed_monitor(pattern = property.PROPERTY)
	if 'time' in event_dict:
		if last_time is None:
			last_time = event_dict['time']
			event_dict['time'] = 0
		else:
			event_dict['time'] = event_dict['time'] - last_time
	
	###############################################
	# added by Maryam	to solve the following error:
	# File "./oracle.py", line 82, in check_event
    	# res = tl_oracle.update(abs_msg)
	# RuntimeError: Unable to cast Python instance to C++ type (compile in debug mode for details)
	# cause: numbers cannot be case to C++ type, convert to string
	# BEGIN
	#print('***', event_dict) ### added by Maryam
	for key, val in event_dict.items(): ### added by Maryam
		event_dict[key] = str(val) ### added by Maryam
	# END
	###############################################
		 
	abs_msg = property.abstract_message(event_dict)
	res = tl_oracle.update(abs_msg)
	if model == 'discrete' and res:
		last_res = res['value']
	if model == 'dense' and res:
		last_res = True
		for d in res:
			if not d['value']:
				last_res = False
				break
	return last_res

def main(argv):
	global property, tl_oracle, last_time, model, last_res

	parser = argparse.ArgumentParser(
        description='this is an Oracle Python implementation based on Reelay for monitoring PTL, MTL and STL properties',
        formatter_class=argparse.RawTextHelpFormatter)
	parser.add_argument('--property',
		help='Python file describing the property, the predicates and how to translate JSON messages to high-level predicate representations',
		default = 'property',
		type = str)
	parser.add_argument('--online',
		action='store_true')
	parser.add_argument('--offline',
		action='store_true')
	parser.add_argument('--discrete',
		action='store_true')
	parser.add_argument('--dense',
		action='store_true')
	parser.add_argument('--port',
		help='Port where the Websocket Oracle has to listen on',
		default = 8080,
		type = int)
	parser.add_argument('--trace',
		help='File to analyse containing a trace of events generated by a previous execution of the system',
		type = str)
	args = parser.parse_args()

	property = importlib.import_module(args.property)
	last_time = None
	tl_oracle = None
	last_res = True

	if args.discrete:
		model = 'discrete'
	elif args.dense:
		model = 'dense'
	else:
		print('You have to specify if the time is discrete or dense (example, --discrete, -- dense)')
		return

	if args.online:
		# init Websocket
		server = WebsocketServer(args.port)
		server.set_fn_new_client(new_client)
		server.set_fn_client_left(client_left)
		server.set_fn_message_received(message_received)
		server.run_forever()
	elif args.offline:
		if args.trace is None:
			print('For offline verification you have to specify the file containing the trace to analyse (example, --trace <path_to_file>)')
			return
		with open(args.trace, 'r') as log_file:
			trace = log_file.readlines()
			for event in trace:
				if not check_event(event):
					print('Property violated: ' + property.PROPERTY)
					print('The event: \n' + event + '\n' + 'is not consistent with the specification.\n')
					#return
				else:
					print('Property satisfied: ' + property.PROPERTY)
					print('The event: \n' + event + '\n' + 'is consistent with the specification.\n')	
	else:
		print('You have to specify if the oracle has to perform Online (--online) or Offline (--offline) verification')

if __name__ == '__main__':
	main(sys.argv)
