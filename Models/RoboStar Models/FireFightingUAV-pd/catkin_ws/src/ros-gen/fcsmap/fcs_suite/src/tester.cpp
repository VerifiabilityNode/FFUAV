#include <memory>
#include "event.hpp"
#include "messages.hpp"
#include "tester.hpp"
#include "traceexecutor.hpp"

// Callbacks for every topic published by the node under test
void Tester::fcs_battery_stateCallback(uav_msgs::BatteryPercentage::ConstPtr message) {
	auto omsg = std::make_shared<fcs_battery_stateMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for fcs_battery_state.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
// Acknowledgment callbacks
void Tester::gps_position_acceptedCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<gps_position_acceptedMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for gps_position_accepted.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
void Tester::dji_battery_state_acceptedCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<dji_battery_state_acceptedMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for dji_battery_state_accepted.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
void Tester::height_above_takeoff_acceptedCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<height_above_takeoff_acceptedMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for height_above_takeoff_accepted.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}

// Callbacks for instrumentation topics for services that we act as a client.


// Callbacks for instrumentation topics for services that we act as a server.
void Tester::mission_waypoint_upload_requestCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<mission_waypoint_upload_requestMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for mission_waypoint_upload_request.");
	#endif
	exec.put(omsg);
	exec.getSTMessage(omsg);
}
void Tester::drone_task_control_requestCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<drone_task_control_requestMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for drone_task_control_request.");
	#endif
	exec.put(omsg);
	exec.getSTMessage(omsg);
}
void Tester::sdk_control_authority_requestCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<sdk_control_authority_requestMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for sdk_control_authority_request.");
	#endif
	exec.put(omsg);
	exec.getSTMessage(omsg);
}
void Tester::sdk_activation_requestCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<sdk_activation_requestMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for sdk_activation_request.");
	#endif
	exec.put(omsg);
	exec.getSTMessage(omsg);
}
void Tester::mission_waypoint_action_requestCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<mission_waypoint_action_requestMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for mission_waypoint_action_request.");
	#endif
	exec.put(omsg);
	exec.getSTMessage(omsg);
}

void Tester::mission_waypoint_upload_respondCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<mission_waypoint_upload_respondMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for mission_waypoint_upload_respond.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
void Tester::drone_task_control_respondCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<drone_task_control_respondMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for drone_task_control_respond.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
void Tester::sdk_control_authority_respondCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<sdk_control_authority_respondMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for sdk_control_authority_respond.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
void Tester::sdk_activation_respondCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<sdk_activation_respondMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for sdk_activation_respond.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
void Tester::mission_waypoint_action_respondCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<mission_waypoint_action_respondMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for mission_waypoint_action_respond.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}

bool Tester::mission_waypoint_uploadCallback(dji_sdk::MissionWpUpload::Request &request, dji_sdk::MissionWpUpload::Response &response) {
	auto reqM = std::make_shared<mission_waypoint_uploadRequestMessage>(&request);
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for mission_waypoint_upload.");
	#endif
	exec.getTTMessage(reqM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Request for mission_waypoint_upload received, proceeding.");
	#endif
	
	auto resM = std::make_shared<mission_waypoint_uploadResponseMessage>(&response);
	exec.getSTMessage(resM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Responded to service mission_waypoint_upload");
	#endif
	
	return true;
}
bool Tester::drone_task_controlCallback(dji_sdk::DroneTaskControl::Request &request, dji_sdk::DroneTaskControl::Response &response) {
	auto reqM = std::make_shared<drone_task_controlRequestMessage>(&request);
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for drone_task_control.");
	#endif
	exec.getTTMessage(reqM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Request for drone_task_control received, proceeding.");
	#endif
	
	auto resM = std::make_shared<drone_task_controlResponseMessage>(&response);
	exec.getSTMessage(resM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Responded to service drone_task_control");
	#endif
	
	return true;
}
bool Tester::sdk_control_authorityCallback(dji_sdk::SDKControlAuthority::Request &request, dji_sdk::SDKControlAuthority::Response &response) {
	auto reqM = std::make_shared<sdk_control_authorityRequestMessage>(&request);
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for sdk_control_authority.");
	#endif
	exec.getTTMessage(reqM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Request for sdk_control_authority received, proceeding.");
	#endif
	
	auto resM = std::make_shared<sdk_control_authorityResponseMessage>(&response);
	exec.getSTMessage(resM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Responded to service sdk_control_authority");
	#endif
	
	return true;
}
bool Tester::sdk_activationCallback(dji_sdk::Activation::Request &request, dji_sdk::Activation::Response &response) {
	auto reqM = std::make_shared<sdk_activationRequestMessage>(&request);
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for sdk_activation.");
	#endif
	exec.getTTMessage(reqM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Request for sdk_activation received, proceeding.");
	#endif
	
	auto resM = std::make_shared<sdk_activationResponseMessage>(&response);
	exec.getSTMessage(resM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Responded to service sdk_activation");
	#endif
	
	return true;
}
bool Tester::mission_waypoint_actionCallback(dji_sdk::MissionWpAction::Request &request, dji_sdk::MissionWpAction::Response &response) {
	auto reqM = std::make_shared<mission_waypoint_actionRequestMessage>(&request);
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for mission_waypoint_action.");
	#endif
	exec.getTTMessage(reqM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Request for mission_waypoint_action received, proceeding.");
	#endif
	
	auto resM = std::make_shared<mission_waypoint_actionResponseMessage>(&response);
	exec.getSTMessage(resM);
	
	#ifdef DEBUG
		ROS_INFO_STREAM("Responded to service mission_waypoint_action");
	#endif
	
	return true;
}

void Tester::fly_to_wpStatusCallback(actionlib_msgs::GoalStatusArray::ConstPtr message) {
	
	// We only record a status message when it the goal is accepted (input accepted), and only
	// when the status refers to a new goal_id, so that we do not acknowledge the same goal twice.
	
	if (message->status_list.size() > 0) {
	 	if (message->status_list[0].status == actionlib_msgs::GoalStatus::ACTIVE) {
	 		
	 		bool skip = false;
	 		
	 		// It already existed
	 		if (auto prev = fly_to_wp_goal_prev) {
	 			if (prev->goal_id == message->status_list[0].goal_id) {
	 				skip = true;
				}
	 		}
	 		
	 		if (!skip) {
	 			auto omsg = std::make_shared<fly_to_wpStatusMessage>();
	 			auto themsg = boost::make_shared<actionlib_msgs::GoalStatus>();
	 			
	 			// Copy value across
	 			*themsg = message->status_list[0];
	 			// Assign pointer
				omsg->message = themsg;
				#ifdef DEBUG
					ROS_INFO_STREAM("Callback for fly_to_wp/status.");
				#endif
				exec.put(omsg);
				exec.getSMessage(omsg);
			}
			// Retain current message for later comparison
		 	fly_to_wp_goal_prev = message->status_list[0];
	 	}
	}
}

void Tester::fly_to_wpResultCallback(uav_msgs::FlyToWPActionResult::ConstPtr message) {
	auto omsg = std::make_shared<fly_to_wpResultMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for fly_to_wp/result.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
void Tester::special_movementStatusCallback(actionlib_msgs::GoalStatusArray::ConstPtr message) {
	
	// We only record a status message when it the goal is accepted (input accepted), and only
	// when the status refers to a new goal_id, so that we do not acknowledge the same goal twice.
	
	if (message->status_list.size() > 0) {
	 	if (message->status_list[0].status == actionlib_msgs::GoalStatus::ACTIVE) {
	 		
	 		bool skip = false;
	 		
	 		// It already existed
	 		if (auto prev = special_movement_goal_prev) {
	 			if (prev->goal_id == message->status_list[0].goal_id) {
	 				skip = true;
				}
	 		}
	 		
	 		if (!skip) {
	 			auto omsg = std::make_shared<special_movementStatusMessage>();
	 			auto themsg = boost::make_shared<actionlib_msgs::GoalStatus>();
	 			
	 			// Copy value across
	 			*themsg = message->status_list[0];
	 			// Assign pointer
				omsg->message = themsg;
				#ifdef DEBUG
					ROS_INFO_STREAM("Callback for special_movement/status.");
				#endif
				exec.put(omsg);
				exec.getSMessage(omsg);
			}
			// Retain current message for later comparison
		 	special_movement_goal_prev = message->status_list[0];
	 	}
	}
}

void Tester::special_movementResultCallback(uav_msgs::SpecialMovementActionResult::ConstPtr message) {
	auto omsg = std::make_shared<special_movementResultMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for special_movement/result.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
