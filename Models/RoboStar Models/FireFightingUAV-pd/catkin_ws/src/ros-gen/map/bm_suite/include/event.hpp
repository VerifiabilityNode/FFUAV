#ifndef _H_EVENT
#define _H_EVENT

#include "ros/ros.h"

// Services

// Topics
#include <uav_msgs/InputAccepted.h>
#include <uav_msgs/BatteryStatus.h>
#include <uav_msgs/BatteryPercentage.h>

// Actions

class Event {
	public:
		virtual ~Event() = default;
		virtual std::string getName() const { return "Unknown"; };

		virtual bool compare(uav_msgs::InputAccepted::ConstPtr message) { return false; };
		virtual bool compare(uav_msgs::BatteryStatus::ConstPtr message) { return false; };
		virtual bool compare(uav_msgs::BatteryPercentage::ConstPtr message) { return false; };




		virtual void publish() { 
		#ifdef DEBUG
			ROS_INFO_STREAM("publish called on base class of Event."); 
		#endif
		};
		virtual bool isSUTInput() const { return false; };
};

#endif /* _H_EVENT */
