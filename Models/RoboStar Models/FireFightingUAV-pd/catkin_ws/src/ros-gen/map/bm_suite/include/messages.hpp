#ifndef _H_MESSAGES
#define _H_MESSAGES

#include <optional>
#include "ros/ros.h"
#include "message.hpp"
#include "events.hpp"

class battery_statusMessage : public TopicMessage<uav_msgs::BatteryStatus::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "battery_statusMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<batteryStatusOUT>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->stamp;
  	};
};

class input_acceptedMessage : public TopicMessage<uav_msgs::InputAccepted::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "input_acceptedMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<batteryStateIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->stamp;
  	};
};

class battery_stateMessage : public TopicMessage<uav_msgs::BatteryPercentage::ConstPtr> {
  public:
  	std::string getName() const override {
  		return "battery_stateMessage";
  	};
  	
  	bool compare(std::shared_ptr<Event> ev) const override {
  		if (std::dynamic_pointer_cast<batteryStateIN>(ev)) {
  			return TopicMessage::compare(ev);
  		} else {
  			return false;
  		}
  	};
};




#endif /* _H_MESSAGES */
