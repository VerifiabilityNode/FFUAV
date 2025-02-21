#include <memory>
#include "event.hpp"
#include "messages.hpp"
#include "tester.hpp"
#include "traceexecutor.hpp"

// Callbacks for every topic published by the node under test
void Tester::water_status_topicCallback(uav_msgs::WaterStatus::ConstPtr message) {
	auto omsg = std::make_shared<water_status_topicMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for water_status_topic.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
// Acknowledgment callbacks

// Callbacks for instrumentation topics for services that we act as a client.
void Tester::service_water_monitor_request_topicCallback(std_msgs::Time::ConstPtr message) {
	auto omsg = std::make_shared<service_water_monitor_request_topicMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for service_water_monitor_request_topic.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}


// Callbacks for instrumentation topics for services that we act as a server.



