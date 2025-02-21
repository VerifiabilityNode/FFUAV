#ifndef _H_MESSAGES
#define _H_MESSAGES

#include <optional>
#include "ros/ros.h"
#include "message.hpp"
#include "events.hpp"

class fcs_battery_stateMessage : public TopicMessage<uav_msgs::BatteryPercentage::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "fcs_battery_stateMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<batteryStateOutOUT>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->stamp;
  	};
};

class gps_position_acceptedMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "gps_position_acceptedMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<gpsPositionIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
};
class dji_battery_state_acceptedMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "dji_battery_state_acceptedMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<batteryStateInIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
};
class height_above_takeoff_acceptedMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "height_above_takeoff_acceptedMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<heightAboveTakeOffIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
};

class gps_positionMessage : public TopicMessage<sensor_msgs::NavSatFix::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "gps_positionMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<gpsPositionIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};
class dji_battery_stateMessage : public TopicMessage<sensor_msgs::BatteryState::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "dji_battery_stateMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<batteryStateInIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};
class height_above_takeoffMessage : public TopicMessage<std_msgs::Float32::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "height_above_takeoffMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<heightAboveTakeOffIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};

class fly_to_wpStatusMessage : public TopicMessage<actionlib_msgs::GoalStatus::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "fly_to_wpStatusMessage";
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->goal_id.stamp;
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<waypointIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};


class fly_to_wpResultMessage : public TopicMessage<uav_msgs::FlyToWPActionResult::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "fly_to_wpResultMessage";
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->header.stamp;
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<WPSucceededOUT>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};
class special_movementStatusMessage : public TopicMessage<actionlib_msgs::GoalStatus::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "special_movementStatusMessage";
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->goal_id.stamp;
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<navCommandIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};


class special_movementResultMessage : public TopicMessage<uav_msgs::SpecialMovementActionResult::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "special_movementResultMessage";
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->header.stamp;
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<NavSucceededOUT>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};

class mission_waypoint_uploadRequestMessage : public ServiceRequestMessage<dji_sdk::MissionWpUpload::Request> {
  public:
  	using ServiceRequestMessage::ServiceRequestMessage;
  	
  	std::string getName() const override {
  		return "mission_waypoint_uploadRequestMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcMissionWPUpload>(ev)) {
  			return ServiceRequestMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};

class mission_waypoint_uploadResponseMessage : public ServiceResponseMessage<dji_sdk::MissionWpUpload::Response> {
  public:
  	using ServiceResponseMessage::ServiceResponseMessage;
  	
  	std::string getName() const override {
  		return "mission_waypoint_uploadResponseMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcMissionWPUploadRetIN>(ev)) {
  			return ServiceResponseMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};
class drone_task_controlRequestMessage : public ServiceRequestMessage<dji_sdk::DroneTaskControl::Request> {
  public:
  	using ServiceRequestMessage::ServiceRequestMessage;
  	
  	std::string getName() const override {
  		return "drone_task_controlRequestMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcDroneTaskControl>(ev)) {
  			return ServiceRequestMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};

class drone_task_controlResponseMessage : public ServiceResponseMessage<dji_sdk::DroneTaskControl::Response> {
  public:
  	using ServiceResponseMessage::ServiceResponseMessage;
  	
  	std::string getName() const override {
  		return "drone_task_controlResponseMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcTaskRetIN>(ev)) {
  			return ServiceResponseMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};
class sdk_control_authorityRequestMessage : public ServiceRequestMessage<dji_sdk::SDKControlAuthority::Request> {
  public:
  	using ServiceRequestMessage::ServiceRequestMessage;
  	
  	std::string getName() const override {
  		return "sdk_control_authorityRequestMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcControlAuth>(ev)) {
  			return ServiceRequestMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};

class sdk_control_authorityResponseMessage : public ServiceResponseMessage<dji_sdk::SDKControlAuthority::Response> {
  public:
  	using ServiceResponseMessage::ServiceResponseMessage;
  	
  	std::string getName() const override {
  		return "sdk_control_authorityResponseMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcControlAuthRetIN>(ev)) {
  			return ServiceResponseMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};
class sdk_activationRequestMessage : public ServiceRequestMessage<dji_sdk::Activation::Request> {
  public:
  	using ServiceRequestMessage::ServiceRequestMessage;
  	
  	std::string getName() const override {
  		return "sdk_activationRequestMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcActivation>(ev)) {
  			return ServiceRequestMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};

class sdk_activationResponseMessage : public ServiceResponseMessage<dji_sdk::Activation::Response> {
  public:
  	using ServiceResponseMessage::ServiceResponseMessage;
  	
  	std::string getName() const override {
  		return "sdk_activationResponseMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcActivationRetIN>(ev)) {
  			return ServiceResponseMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};
class mission_waypoint_actionRequestMessage : public ServiceRequestMessage<dji_sdk::MissionWpAction::Request> {
  public:
  	using ServiceRequestMessage::ServiceRequestMessage;
  	
  	std::string getName() const override {
  		return "mission_waypoint_actionRequestMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcMissionWPAction>(ev)) {
  			return ServiceRequestMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};

class mission_waypoint_actionResponseMessage : public ServiceResponseMessage<dji_sdk::MissionWpAction::Response> {
  public:
  	using ServiceResponseMessage::ServiceResponseMessage;
  	
  	std::string getName() const override {
  		return "mission_waypoint_actionResponseMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcMissionWPActionRetIN>(ev)) {
  			return ServiceResponseMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};

// Output (NUT is client)
class mission_waypoint_upload_requestMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "mission_waypoint_upload_requestMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcMissionWPUpload>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match mission_waypoint_upload_requestMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<mission_waypoint_upload_requestMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match mission_waypoint_upload_requestMessage.");
	  		#endif
	  		return std::make_shared<mission_waypoint_uploadRequestMessage>(m->getTime(),nullptr);
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
class mission_waypoint_upload_respondMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "mission_waypoint_upload_respondMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcMissionWPUploadRetIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match mission_waypoint_upload_respondMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<mission_waypoint_upload_respondMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match mission_waypoint_upload_respondMessage.");
	  		#endif
	  		return m;
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
// Output (NUT is client)
class drone_task_control_requestMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "drone_task_control_requestMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcDroneTaskControl>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match drone_task_control_requestMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<drone_task_control_requestMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match drone_task_control_requestMessage.");
	  		#endif
	  		return std::make_shared<drone_task_controlRequestMessage>(m->getTime(),nullptr);
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
class drone_task_control_respondMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "drone_task_control_respondMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcTaskRetIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match drone_task_control_respondMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<drone_task_control_respondMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match drone_task_control_respondMessage.");
	  		#endif
	  		return m;
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
// Output (NUT is client)
class sdk_control_authority_requestMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "sdk_control_authority_requestMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcControlAuth>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match sdk_control_authority_requestMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<sdk_control_authority_requestMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match sdk_control_authority_requestMessage.");
	  		#endif
	  		return std::make_shared<sdk_control_authorityRequestMessage>(m->getTime(),nullptr);
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
class sdk_control_authority_respondMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "sdk_control_authority_respondMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcControlAuthRetIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match sdk_control_authority_respondMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<sdk_control_authority_respondMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match sdk_control_authority_respondMessage.");
	  		#endif
	  		return m;
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
// Output (NUT is client)
class sdk_activation_requestMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "sdk_activation_requestMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcActivation>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match sdk_activation_requestMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<sdk_activation_requestMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match sdk_activation_requestMessage.");
	  		#endif
	  		return std::make_shared<sdk_activationRequestMessage>(m->getTime(),nullptr);
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
class sdk_activation_respondMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "sdk_activation_respondMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcActivationRetIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match sdk_activation_respondMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<sdk_activation_respondMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match sdk_activation_respondMessage.");
	  		#endif
	  		return m;
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
// Output (NUT is client)
class mission_waypoint_action_requestMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "mission_waypoint_action_requestMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcMissionWPAction>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match mission_waypoint_action_requestMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<mission_waypoint_action_requestMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match mission_waypoint_action_requestMessage.");
	  		#endif
	  		return std::make_shared<mission_waypoint_actionRequestMessage>(m->getTime(),nullptr);
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};
class mission_waypoint_action_respondMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "mission_waypoint_action_respondMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<fcMissionWPActionRetIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
  		#ifdef DEBUG
  		ROS_INFO_STREAM("Trying to match mission_waypoint_action_respondMessage.");
  		#endif
  		
  		if (std::dynamic_pointer_cast<mission_waypoint_action_respondMessage>(m)) {
  			#ifdef DEBUG
	  		ROS_INFO_STREAM("Match successful for match mission_waypoint_action_respondMessage.");
	  		#endif
	  		return m;
  		} else {
  			return std::nullopt;
  		}
  		
  	};
};

#endif /* _H_MESSAGES */
