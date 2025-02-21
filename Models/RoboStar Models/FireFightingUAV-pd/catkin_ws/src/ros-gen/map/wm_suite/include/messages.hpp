#ifndef _H_MESSAGES
#define _H_MESSAGES

#include <optional>
#include "ros/ros.h"
#include "message.hpp"
#include "events.hpp"

class water_status_topicMessage : public TopicMessage<uav_msgs::WaterStatus::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "water_status_topicMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<waterStatusOutOUT>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->header.stamp;
  	};
};






// Input (NUT is server)
class service_water_monitor_request_topicMessage : public TopicMessage<std_msgs::Time::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "service_water_monitor_request_topicMessage";
  	};
  	std::optional<ros::Time> getTime() const override {
  		return message->data;
  	};
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<sprayInIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};
#endif /* _H_MESSAGES */
