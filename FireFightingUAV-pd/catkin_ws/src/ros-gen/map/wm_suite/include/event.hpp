#ifndef _H_EVENT
#define _H_EVENT

#include "ros/ros.h"

// Services
#include <uav_msgs/EnableWaterMonitor.h>

// Topics
#include <uav_msgs/WaterStatus.h>
#include <std_msgs/Time.h>

// Actions

class Event {
	public:
		virtual ~Event() = default;
		virtual std::string getName() const { return "Unknown"; };

		virtual bool compare(uav_msgs::WaterStatus::ConstPtr message) { return false; };
		virtual bool compare(std_msgs::Time::ConstPtr message) { return false; };

		virtual bool compare(uav_msgs::EnableWaterMonitor::Request* request) { return false; };
		virtual bool compare(uav_msgs::EnableWaterMonitor::Response* response) { return false; };



		virtual void publish() { 
		#ifdef DEBUG
			ROS_INFO_STREAM("publish called on base class of Event."); 
		#endif
		};
		virtual bool isSUTInput() const { return false; };
};

#endif /* _H_EVENT */
