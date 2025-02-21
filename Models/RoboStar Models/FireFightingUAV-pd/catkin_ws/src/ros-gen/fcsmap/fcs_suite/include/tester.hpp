#ifndef _H_TESTER
#define _H_TESTER

#include <memory>
#include "ros/ros.h"
#include "events.hpp"
#include "verdict.hpp"
#include "traceexecutor.hpp"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalStatusArray.h"

class Tester {
  private:
    std::vector<std::shared_ptr<Message>> events;
	Verdict verdict = Verdict::Inconclusive;
	TraceExecutor exec;
	
	ros::NodeHandle nh;
	
  public:
  
	// NUT Publishers that we subscribe
	ros::Subscriber fcs_battery_state_sub = nh.subscribe("fcs_interface/battery_state", 10, &Tester::fcs_battery_stateCallback, this);
	
	// NUT Instrumentation that we subscribe
	ros::Subscriber gps_position_accepted_sub = nh.subscribe("/fcs_interface/gps_position_accepted", 10, &Tester::gps_position_acceptedCallback, this);
	ros::Subscriber dji_battery_state_accepted_sub = nh.subscribe("/fcs_interface/dji_battery_state_accepted", 10, &Tester::dji_battery_state_acceptedCallback, this);
	ros::Subscriber height_above_takeoff_accepted_sub = nh.subscribe("/fcs_interface/height_above_takeoff_accepted", 10, &Tester::height_above_takeoff_acceptedCallback, this);
	ros::Subscriber sdk_control_authority_request_sub = nh.subscribe("fcs_interface/sdk_control_authority_request", 10, &Tester::sdk_control_authority_requestCallback, this);
	ros::Subscriber drone_task_control_request_sub = nh.subscribe("fcs_interface/drone_task_control_request", 10, &Tester::drone_task_control_requestCallback, this);
	ros::Subscriber sdk_activation_request_sub = nh.subscribe("fcs_interface/sdk_activation_request", 10, &Tester::sdk_activation_requestCallback, this);
	ros::Subscriber mission_waypoint_upload_request_sub = nh.subscribe("fcs_interface/mission_waypoint_upload_request", 10, &Tester::mission_waypoint_upload_requestCallback, this);
	ros::Subscriber mission_waypoint_action_request_sub = nh.subscribe("fcs_interface/mission_waypoint_action_request", 10, &Tester::mission_waypoint_action_requestCallback, this);
	ros::Subscriber sdk_control_authority_respond_sub = nh.subscribe("fcs_interface/sdk_control_authority_respond", 10, &Tester::sdk_control_authority_respondCallback, this);
	ros::Subscriber drone_task_control_respond_sub = nh.subscribe("fcs_interface/drone_task_control_respond", 10, &Tester::drone_task_control_respondCallback, this);
	ros::Subscriber sdk_activation_respond_sub = nh.subscribe("fcs_interface/sdk_activation_respond", 10, &Tester::sdk_activation_respondCallback, this);
	ros::Subscriber mission_waypoint_upload_respond_sub = nh.subscribe("fcs_interface/mission_waypoint_upload_respond", 10, &Tester::mission_waypoint_upload_respondCallback, this);
	ros::Subscriber mission_waypoint_action_respond_sub = nh.subscribe("fcs_interface/mission_waypoint_action_respond", 10, &Tester::mission_waypoint_action_respondCallback, this);
	
	// NUT Subscribers that we publish on
	ros::Publisher gps_position_pub = nh.advertise<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 1000);
	ros::Publisher dji_battery_state_pub = nh.advertise<sensor_msgs::BatteryState>("dji_sdk/battery_state", 1000);
	ros::Publisher height_above_takeoff_pub = nh.advertise<std_msgs::Float32>("dji_sdk/height_above_takeoff", 1000);
	
	// Services that the NUT provides and that we therefore use as a client
	
	// Services that the NUT calls and that we therefore provide as a server
	ros::ServiceServer mission_waypoint_upload_server = nh.advertiseService("dji_sdk/mission_waypoint_upload", &Tester::mission_waypoint_uploadCallback, this);
	ros::ServiceServer drone_task_control_server = nh.advertiseService("dji_sdk/drone_task_control", &Tester::drone_task_controlCallback, this);
	ros::ServiceServer sdk_control_authority_server = nh.advertiseService("dji_sdk/sdk_control_authority", &Tester::sdk_control_authorityCallback, this);
	ros::ServiceServer sdk_activation_server = nh.advertiseService("dji_sdk/activation", &Tester::sdk_activationCallback, this);
	ros::ServiceServer mission_waypoint_action_server = nh.advertiseService("dji_sdk/mission_waypoint_action", &Tester::mission_waypoint_actionCallback, this);
	
	// Callbacks for '/status', '/feedback' and '/result' of action servers implemented by the node under test		    
    ros::Publisher fly_to_wp_goal_pub = nh.advertise<uav_msgs::FlyToWPActionGoal>("/fcs_interface/fly_to_wp/goal", 1000);
    ros::Subscriber fly_to_wp_status_sub = nh.subscribe("/fcs_interface/fly_to_wp/status", 10, &Tester::fly_to_wpStatusCallback, this);
    
    // Keep track of previous goal status message
    std::optional<actionlib_msgs::GoalStatus> fly_to_wp_goal_prev;
    
    ros::Subscriber fly_to_wp_result_sub = nh.subscribe("/fcs_interface/fly_to_wp/result", 10, &Tester::fly_to_wpResultCallback, this);
    ros::Publisher special_movement_goal_pub = nh.advertise<uav_msgs::SpecialMovementActionGoal>("/fcs_interface/special_movement/goal", 1000);
    ros::Subscriber special_movement_status_sub = nh.subscribe("/fcs_interface/special_movement/status", 10, &Tester::special_movementStatusCallback, this);
    
    // Keep track of previous goal status message
    std::optional<actionlib_msgs::GoalStatus> special_movement_goal_prev;
    
    ros::Subscriber special_movement_result_sub = nh.subscribe("/fcs_interface/special_movement/result", 10, &Tester::special_movementResultCallback, this);
	
	void ready() {
		bool ready = false;

		while (!ready) {
			ready = fcs_battery_state_sub.getNumPublishers() > 0
					 &&
					gps_position_accepted_sub.getNumPublishers() > 0 &&
					dji_battery_state_accepted_sub.getNumPublishers() > 0 &&
					height_above_takeoff_accepted_sub.getNumPublishers() > 0 &&
					sdk_control_authority_request_sub.getNumPublishers() > 0 &&
					drone_task_control_request_sub.getNumPublishers() > 0 &&
					sdk_activation_request_sub.getNumPublishers() > 0 &&
					mission_waypoint_upload_request_sub.getNumPublishers() > 0 &&
					mission_waypoint_action_request_sub.getNumPublishers() > 0 &&
					sdk_control_authority_respond_sub.getNumPublishers() > 0 &&
					drone_task_control_respond_sub.getNumPublishers() > 0 &&
					sdk_activation_respond_sub.getNumPublishers() > 0 &&
					mission_waypoint_upload_respond_sub.getNumPublishers() > 0 &&
					mission_waypoint_action_respond_sub.getNumPublishers() > 0
					 &&
					fly_to_wp_status_sub.getNumPublishers() > 0 &&
					special_movement_status_sub.getNumPublishers() > 0
					;
		}
	};
	
	ros::Timer regular_timer = nh.createTimer(ros::Duration(0.2), &Tester::_periodicTimeCallback, this, false, true);
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
    void fcs_battery_stateCallback(uav_msgs::BatteryPercentage::ConstPtr message);
    
    // Acknowledgment callbacks
    void gps_position_acceptedCallback(std_msgs::Time::ConstPtr message);
    void dji_battery_state_acceptedCallback(std_msgs::Time::ConstPtr message);
    void height_above_takeoff_acceptedCallback(std_msgs::Time::ConstPtr message);
    void sdk_control_authority_requestCallback(std_msgs::Time::ConstPtr message);
    void drone_task_control_requestCallback(std_msgs::Time::ConstPtr message);
    void sdk_activation_requestCallback(std_msgs::Time::ConstPtr message);
    void mission_waypoint_upload_requestCallback(std_msgs::Time::ConstPtr message);
    void mission_waypoint_action_requestCallback(std_msgs::Time::ConstPtr message);
    void sdk_control_authority_respondCallback(std_msgs::Time::ConstPtr message);
    void drone_task_control_respondCallback(std_msgs::Time::ConstPtr message);
    void sdk_activation_respondCallback(std_msgs::Time::ConstPtr message);
    void mission_waypoint_upload_respondCallback(std_msgs::Time::ConstPtr message);
    void mission_waypoint_action_respondCallback(std_msgs::Time::ConstPtr message);
   	
    // Service callbacks for every service called by the node under test
    bool mission_waypoint_uploadCallback(dji_sdk::MissionWpUpload::Request &request, dji_sdk::MissionWpUpload::Response &response);
    bool drone_task_controlCallback(dji_sdk::DroneTaskControl::Request &request, dji_sdk::DroneTaskControl::Response &response);
    bool sdk_control_authorityCallback(dji_sdk::SDKControlAuthority::Request &request, dji_sdk::SDKControlAuthority::Response &response);
    bool sdk_activationCallback(dji_sdk::Activation::Request &request, dji_sdk::Activation::Response &response);
    bool mission_waypoint_actionCallback(dji_sdk::MissionWpAction::Request &request, dji_sdk::MissionWpAction::Response &response);
	
	// Callbacks for '/status', '/feedback' and '/result' of action servers implemented by the node under test		    
    void fly_to_wpStatusCallback(actionlib_msgs::GoalStatusArray::ConstPtr message);
    
    void fly_to_wpResultCallback(uav_msgs::FlyToWPActionResult::ConstPtr message);
    void special_movementStatusCallback(actionlib_msgs::GoalStatusArray::ConstPtr message);
    
    void special_movementResultCallback(uav_msgs::SpecialMovementActionResult::ConstPtr message);
    
    // Setup events
    void setup(std::shared_ptr<Event> event) {
    	if (auto ev = std::dynamic_pointer_cast<gpsPositionIN>(event)) {
    		auto adapter = std::make_shared<TopicInputAdapter<sensor_msgs::NavSatFix>>(&gps_position_pub);
    		ev->setAdapter(adapter);
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<batteryStateInIN>(event)) {
    		auto adapter = std::make_shared<TopicInputAdapter<sensor_msgs::BatteryState>>(&dji_battery_state_pub);
    		ev->setAdapter(adapter);
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<heightAboveTakeOffIN>(event)) {
    		auto adapter = std::make_shared<TopicInputAdapter<std_msgs::Float32>>(&height_above_takeoff_pub);
    		ev->setAdapter(adapter);
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<fcControlAuthRetIN>(event)) {
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<fcTaskRetIN>(event)) {
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<fcActivationRetIN>(event)) {
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<fcMissionWPUploadRetIN>(event)) {
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<fcMissionWPActionRetIN>(event)) {
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<waypointIN>(event)) {
    		auto adapter = std::make_shared<TopicInputAdapter<uav_msgs::FlyToWPActionGoal>>(&fly_to_wp_goal_pub);
    		ev->setAdapter(adapter);
    	} else 
    	if (auto ev = std::dynamic_pointer_cast<navCommandIN>(event)) {
    		auto adapter = std::make_shared<TopicInputAdapter<uav_msgs::SpecialMovementActionGoal>>(&special_movement_goal_pub);
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
