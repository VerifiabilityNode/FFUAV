#ifndef _H_TESTER
#define _H_TESTER

#include <memory>
#include "ros/ros.h"
#include "events.hpp"
#include "verdict.hpp"
#include "traceexecutor.hpp"

class Tester {
  private:
    std::vector<std::shared_ptr<Message>> events;
	Verdict verdict = Verdict::Inconclusive;
	TraceExecutor exec;
	
	ros::NodeHandle nh;
	
  public:
  
	// NUT Publishers that we subscribe
	ros::Subscriber water_status_topic_sub = nh.subscribe("water_monitor_node/water_status_topic", 10, &Tester::water_status_topicCallback, this);
	
	// NUT Instrumentation that we subscribe
	ros::Subscriber service_water_monitor_request_topic_sub = nh.subscribe("water_monitor_node/service_water_monitor_request_topic", 10, &Tester::service_water_monitor_request_topicCallback, this);
	
	// NUT Subscribers that we publish on
	
	// Services that the NUT provides and that we therefore use as a client
	ros::ServiceClient enable_water_monitor_client = nh.serviceClient<uav_msgs::EnableWaterMonitor>("EnableWaterMonitor");
	
	// Services that the NUT calls and that we therefore provide as a server
	
	// Callbacks for '/status', '/feedback' and '/result' of action servers implemented by the node under test		    
	
	void ready() {
		bool ready = false;

		while (!ready) {
			ready = water_status_topic_sub.getNumPublishers() > 0
					 &&
					service_water_monitor_request_topic_sub.getNumPublishers() > 0
					 &&
					enable_water_monitor_client.waitForExistence()
					;
		}
	};
	
	ros::Timer regular_timer = nh.createTimer(ros::Duration(0.5), &Tester::_periodicTimeCallback, this, false, true);
	ros::Timer final_timer;
	
  	// Timer callbacks
  	void _finalTimeoutCallback(const ros::TimerEvent& event) {
  		ROS_INFO_STREAM("Final timeout reached.");
  		exec.shutdown();
  	};
  	void _periodicTimeCallback(const ros::TimerEvent& event) {
  		exec.update();
  	};
  	
    // Callbacks for every topic published by the node under test
    void water_status_topicCallback(uav_msgs::WaterStatus::ConstPtr message);
    
    // Acknowledgment callbacks
    void service_water_monitor_request_topicCallback(std_msgs::Time::ConstPtr message);
   	
    // Service callbacks for every service called by the node under test
	
	// Callbacks for '/status', '/feedback' and '/result' of action servers implemented by the node under test		    
    
    // Setup events
    void setup(std::shared_ptr<Event> event) {
    	if (auto ev = std::dynamic_pointer_cast<sprayInIN>(event)) {
    		auto adapter = std::make_shared<AsyncServiceInputAdapter<uav_msgs::EnableWaterMonitor::Request,uav_msgs::EnableWaterMonitor::Response>>(&enable_water_monitor_client, &exec);
    		ev->setAdapter(adapter);
    	}
    };
    

    // Standard methods
    Verdict getVerdict() { return verdict; };
    
    void setTrace(std::vector<std::shared_ptr<Event>> ex) {
		exec.setTrace(ex);
		final_timer = nh.createTimer(ros::Duration(1+0.5*ex.size())+ros::Duration(1), &Tester::_finalTimeoutCallback, this, true, true);
	};
	std::vector<std::shared_ptr<Message>> getMessages() {
		return exec.getMessages();
	};
	TraceExecutor* getExecutor() {
		return &exec;
	};
};

#endif
