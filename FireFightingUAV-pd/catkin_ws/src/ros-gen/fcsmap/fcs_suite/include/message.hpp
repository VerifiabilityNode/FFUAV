#ifndef _H_MESSAGE
#define _H_MESSAGE

#include <memory>
#include <optional>
#include <string>
#include "event.hpp"
#include "ros/ros.h"

class Message {
  protected:
	std::optional<ros::Time> time = std::nullopt;
  public:
	virtual std::optional<ros::Time> getTime() const { return time; };
	virtual void setTime(ros::Time t) { time = t; };

	virtual std::string getName() const {
		return "Unknown";
	};

	virtual std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
		#ifdef DEBUG
		ROS_INFO_STREAM("No match for Message");
		#endif
		return std::nullopt;
	};

	/*
	 * Compares the supplied Event with this message, returning true
	 * if they match. This method is used to check a trace of events
	 * against a trace of messages.
	 */
	virtual bool compare(const Event& ev) const { return false; };
	virtual void respond(const Event& ev) const { };

	virtual bool compare(std::shared_ptr<Event> ev) const { return false; };
	virtual std::string getFullName() {
		if (getTime().has_value()) {
			return getName() + " (" + std::to_string(getTime().value().sec) + "." + std::to_string(getTime().value().nsec) + ")";
		} else {
			return getName() + " ()";
		}
	};
};

bool LTHAN(std::shared_ptr<Message> m1, std::shared_ptr<Message> m2);

// Template for ROS Topic Messages
template <typename M>
class TopicMessage : public Message {
  public:
	M message;

	std::string getName() const override {
	  return "Unknown TopicMessage";
	};
//	bool compare(const Event& ev) const {
//	  return ev.compare(message);
//	}
	bool compare(std::shared_ptr<Event> ev) const {
		return ev->compare(message);
	}
	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
	  if (std::dynamic_pointer_cast<TopicMessage<M>>(m)) {
		#ifdef DEBUG
		ROS_INFO_STREAM("Match successful for TopicMessage<M>");
		#endif
		return m;
	  } else {
		return std::nullopt;
	  }
	};
};

template <typename M>
class ServiceMessage : public Message {
  public:
	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
	  if (std::dynamic_pointer_cast<ServiceMessage<M>>(m)) {
		#ifdef DEBUG
		ROS_INFO_STREAM("Match successful for ServiceMessage<M>");
		#endif
		auto time = this->getTime();
		if (time) {
			m->setTime(time.value());
		}
		return m;
	  } else {
		return std::nullopt;
	  }
	};
};

template <typename M>
class ServiceRequestMessage : public ServiceMessage<M>  {
  private:
    M* request;
  public:
    ServiceRequestMessage(std::optional<ros::Time> _time, M* _request) {
		this->time = _time;
		this->request = _request;
	};
    ServiceRequestMessage(M* _request) {
    	this->request = _request;
    };
    M* getRequest() {
    	return request;
    }
    bool compare(std::shared_ptr<Event> ev) const {
		#ifdef DEBUG
    	ROS_INFO("Hit compare of '%s' as a ServiceRequestMessage.", this->getName().c_str());
		#endif
    	if (request != nullptr) {
			return ev->compare(request);
		} else {
			return false;
		}
    };
    std::string getName() const override {
      return "ServiceRequestMessage";
    };
};

template <typename M>
class ServiceResponseMessage : public ServiceMessage<M> {
  private:
    M* response;
  public:
	ServiceResponseMessage(std::optional<ros::Time> _time, M* _response) {
		this->time = _time;
		this->response = _response;
	}
	ServiceResponseMessage(M* _response) {
		this->response = _response;
	}
	M* getResponse() {
		return response;
	}
	bool compare(std::shared_ptr<Event> ev) const override {
		#ifdef DEBUG
			ROS_INFO("Hit compare of %s.", this->getName().c_str());
		#endif
		if (response != nullptr) {
			return ev->compare(response);
		} else {
			return false;
		}
	}
	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) override {
		#ifdef DEBUG
			ROS_INFO("Trying to match %s.", this->getName().c_str());
		#endif
      if (auto mcast = std::dynamic_pointer_cast<ServiceResponseMessage<M>>(m)) {
		#ifdef DEBUG
    	  ROS_INFO("Match successful for %s.", this->getName().c_str());
		#endif
        return std::make_shared<ServiceResponseMessage<M>>(this->time,mcast->getResponse());
      } else {
        return std::nullopt;
      }
    };
	std::string getName() const override {
		return "ServiceResponseMessage";
	};
};

#endif /* _H_MESSAGE */
