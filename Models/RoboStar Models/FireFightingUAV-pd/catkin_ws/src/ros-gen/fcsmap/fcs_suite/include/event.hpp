#ifndef _H_EVENT
#define _H_EVENT

#include "ros/ros.h"

// Services
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/SetLocalPosRef.h>

// Topics
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <uav_msgs/BatteryPercentage.h>
#include <sensor_msgs/Joy.h>

// Actions
#include <uav_msgs/FlyToWPAction.h>
#include <uav_msgs/SpecialMovementAction.h>
#include <uav_msgs/RelativePositionAction.h>

class Event {
	public:
		virtual ~Event() = default;
		virtual std::string getName() const { return "Unknown"; };

		virtual bool compare(sensor_msgs::NavSatFix::ConstPtr message) { return false; };
		virtual bool compare(std_msgs::Time::ConstPtr message) { return false; };
		virtual bool compare(sensor_msgs::BatteryState::ConstPtr message) { return false; };
		virtual bool compare(geometry_msgs::PointStamped::ConstPtr message) { return false; };
		virtual bool compare(std_msgs::UInt8::ConstPtr message) { return false; };
		virtual bool compare(std_msgs::Float32::ConstPtr message) { return false; };
		virtual bool compare(uav_msgs::BatteryPercentage::ConstPtr message) { return false; };
		virtual bool compare(sensor_msgs::Joy::ConstPtr message) { return false; };

		virtual bool compare(dji_sdk::SDKControlAuthority::Request* request) { return false; };
		virtual bool compare(dji_sdk::SDKControlAuthority::Response* response) { return false; };
		virtual bool compare(dji_sdk::Activation::Request* request) { return false; };
		virtual bool compare(dji_sdk::Activation::Response* response) { return false; };
		virtual bool compare(dji_sdk::DroneTaskControl::Request* request) { return false; };
		virtual bool compare(dji_sdk::DroneTaskControl::Response* response) { return false; };
		virtual bool compare(dji_sdk::MissionWpAction::Request* request) { return false; };
		virtual bool compare(dji_sdk::MissionWpAction::Response* response) { return false; };
		virtual bool compare(dji_sdk::MissionWpUpload::Request* request) { return false; };
		virtual bool compare(dji_sdk::MissionWpUpload::Response* response) { return false; };
		virtual bool compare(dji_sdk::SetLocalPosRef::Request* request) { return false; };
		virtual bool compare(dji_sdk::SetLocalPosRef::Response* response) { return false; };

		virtual bool compare(actionlib_msgs::GoalStatus::ConstPtr message) { return false; };

		virtual bool compare(uav_msgs::FlyToWPActionFeedback::ConstPtr message) { return false; };
		virtual bool compare(uav_msgs::FlyToWPActionResult::ConstPtr message) { return false; };
		virtual bool compare(uav_msgs::SpecialMovementActionFeedback::ConstPtr message) { return false; };
		virtual bool compare(uav_msgs::SpecialMovementActionResult::ConstPtr message) { return false; };
		virtual bool compare(uav_msgs::RelativePositionActionFeedback::ConstPtr message) { return false; };
		virtual bool compare(uav_msgs::RelativePositionActionResult::ConstPtr message) { return false; };

		virtual void publish() { 
		#ifdef DEBUG
			ROS_INFO_STREAM("publish called on base class of Event."); 
		#endif
		};
		virtual bool isSUTInput() const { return false; };
};

#endif /* _H_EVENT */
