import oracle

Mission_Threshold = 40
Safety_Threshold = 30

battery_state_topic = '/fcs_interface/battery_state'
input_accepted_topic = '/battery_monitor/input_accepted'
battery_status_topic = '/battery_monitor/battery_status'


pl = [
'(forall[i]. {topic: "'+input_accepted_topic+'", id: *i} -> once({topic: "'+battery_state_topic+'", id: *i}))',
'(forall[i]. {topic: "'+battery_status_topic+'", id: *i} -> once({topic: "'+input_accepted_topic+'", id: *i}))',
      '(forall[i]. (forall[s]. {topic: "'+battery_status_topic+'", status: *s, id: *i} -> once({topic: "'+battery_state_topic+'", "percentage": *s, id: *i})))',
      '(forall[i]. {topic: "'+input_accepted_topic+'", id: *i} -> (historically[1:](not {topic: "'+input_accepted_topic+'", id: *i})) and (forall[j]. (historically[1:] (not {topic: "'+input_accepted_topic+'", id: *j})) or (once({topic: "'+battery_status_topic+'", id: *j}))))',
'(forall[i]. ({topic: "'+battery_status_topic+'", id: *i} -> (historically[1:] (not {topic: "'+battery_status_topic+'", id: *i}))))'
]

# property to verify
PROPERTY = ' and '.join(pl)

# predicates used in the property (initialization for time 0)
predicates = dict()



# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    print(message)

    predicates['topic'] = message['topic']
    predicates['id'] = str(message['input_msg_id'])
    #predicates['secs'] = str(eval(message['stamp'])['secs']) 
    #predicates['nsecs'] = str(eval(message['stamp'])['nsecs']) 

    if message['topic'] == battery_state_topic:
    	percentage = float(message['percentage'])
    	if percentage >= 0 and percentage <= 100: 
    		if percentage > Mission_Threshold:
    			predicates['percentage'] = 'OK'
    		elif percentage > Safety_Threshold:
    			predicates['percentage'] = 'MC'
    		elif percentage >= 0:
    			predicates['percentage'] = 'SC' 
    	else:
    		predicates['percentage'] = 'INVALID'

    if message['topic'] == battery_status_topic:
    	status = int(message['status'])
    	if status == 1:
    		predicates['status'] = 'OK'
    	elif status == 2:
    		predicates['status'] = 'MC'
    	elif status == 3:
    		predicates['status'] = 'SC'
    	elif status == 4:
    		predicates['status'] = 'INVALID'
    	else:
    		predicates['status'] = 'ELSE'
    #print(predicates)
    return predicates
