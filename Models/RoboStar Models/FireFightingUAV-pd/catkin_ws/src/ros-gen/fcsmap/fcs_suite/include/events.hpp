#ifndef _H_EVENTS
#define _H_EVENTS

#include "event.hpp"
#include "eventio.hpp"
#include "inputadapter.hpp"
#include "types.hpp"
#include "primitivetypes.hpp"

class gpsPositionIN : public TypedInputEvent<Position,sensor_msgs::NavSatFix> {
  public:
  	// Inherit constructors
  	using TypedInputEvent::TypedInputEvent;
  	
	// Name
  	std::string getName() const override { return "gpsPosition"; };
  	
  	// User provided implementations
  	sensor_msgs::NavSatFix getROSMessage() override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class fcMissionWPUpload : public TypedOutputEvent<std::tuple<std::vector<Position>>> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "fcMissionWPUpload"; };
  	
  	// User provided implementations
  	bool compare(dji_sdk::MissionWpUpload::Request* message) override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class fcDroneTaskControl : public TypedOutputEvent<std::tuple<M600Task::M600Task>> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "fcDroneTaskControl"; };
  	
  	// User provided implementations
  	bool compare(dji_sdk::DroneTaskControl::Request* message) override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class fcControlAuthRetIN : public TypedResponseInputEvent<bool> {
  public:
  	// Inherit constructors
  	using TypedResponseInputEvent<bool>::TypedResponseInputEvent;
  	
	// Name
  	std::string getName() const override { return "fcControlAuthRet"; };
  	
  	// User provided implementations
  	bool compare(std_msgs::Time::ConstPtr message) override;
  	bool compare(dji_sdk::SDKControlAuthority::Response* message) override;
};
class fcControlAuth : public TypedOutputEvent<std::tuple<>> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "fcControlAuth"; };
  	
  	// User provided implementations
  	bool compare(dji_sdk::SDKControlAuthority::Request* message) override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class batteryStateInIN : public TypedInputEvent<Battery,sensor_msgs::BatteryState> {
  public:
  	// Inherit constructors
  	using TypedInputEvent::TypedInputEvent;
  	
	// Name
  	std::string getName() const override { return "batteryStateIn"; };
  	
  	// User provided implementations
  	sensor_msgs::BatteryState getROSMessage() override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class WPSucceededOUT : public TypedOutputEvent<bool> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "WPSucceeded"; };
  	
  	// User provided implementations
  	bool compare(uav_msgs::FlyToWPActionResult::ConstPtr message) override;
};
class batteryStateOutOUT : public TypedOutputEvent<Battery> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "batteryStateOut"; };
  	
  	// User provided implementations
  	bool compare(uav_msgs::BatteryPercentage::ConstPtr message) override;
};
class fcTaskRetIN : public TypedResponseInputEvent<bool> {
  public:
  	// Inherit constructors
  	using TypedResponseInputEvent<bool>::TypedResponseInputEvent;
  	
	// Name
  	std::string getName() const override { return "fcTaskRet"; };
  	
  	// User provided implementations
  	bool compare(std_msgs::Time::ConstPtr message) override;
  	bool compare(dji_sdk::DroneTaskControl::Response* message) override;
};
class waypointIN : public TypedInputEvent<Position,uav_msgs::FlyToWPActionGoal> {
  public:
  	// Inherit constructors
  	using TypedInputEvent::TypedInputEvent;
  	
	// Name
  	std::string getName() const override { return "waypoint"; };
  	
  	// Constructs ActionGoal message to be published
  	uav_msgs::FlyToWPActionGoal getROSMessage() override {
  		uav_msgs::FlyToWPActionGoal msg;
  		msg.goal = getROSGoalMessage();
  		return msg;
  	};
  	
  	// User provided implementations
  	uav_msgs::FlyToWPGoal getROSGoalMessage();
  	
  	// Handles the acceptance of the goal
  	bool compare(actionlib_msgs::GoalStatus::ConstPtr message) override;
};
class heightAboveTakeOffIN : public TypedInputEvent<int,std_msgs::Float32> {
  public:
  	// Inherit constructors
  	using TypedInputEvent::TypedInputEvent;
  	
	// Name
  	std::string getName() const override { return "heightAboveTakeOff"; };
  	
  	// User provided implementations
  	std_msgs::Float32 getROSMessage() override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class fcActivation : public TypedOutputEvent<std::tuple<>> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "fcActivation"; };
  	
  	// User provided implementations
  	bool compare(dji_sdk::Activation::Request* message) override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class fcActivationRetIN : public TypedResponseInputEvent<bool> {
  public:
  	// Inherit constructors
  	using TypedResponseInputEvent<bool>::TypedResponseInputEvent;
  	
	// Name
  	std::string getName() const override { return "fcActivationRet"; };
  	
  	// User provided implementations
  	bool compare(std_msgs::Time::ConstPtr message) override;
  	bool compare(dji_sdk::Activation::Response* message) override;
};
class fcMissionWPUploadRetIN : public TypedResponseInputEvent<bool> {
  public:
  	// Inherit constructors
  	using TypedResponseInputEvent<bool>::TypedResponseInputEvent;
  	
	// Name
  	std::string getName() const override { return "fcMissionWPUploadRet"; };
  	
  	// User provided implementations
  	bool compare(std_msgs::Time::ConstPtr message) override;
  	bool compare(dji_sdk::MissionWpUpload::Response* message) override;
};
class fcMissionWPActionRetIN : public TypedResponseInputEvent<bool> {
  public:
  	// Inherit constructors
  	using TypedResponseInputEvent<bool>::TypedResponseInputEvent;
  	
	// Name
  	std::string getName() const override { return "fcMissionWPActionRet"; };
  	
  	// User provided implementations
  	bool compare(std_msgs::Time::ConstPtr message) override;
  	bool compare(dji_sdk::MissionWpAction::Response* message) override;
};
class fcMissionWPAction : public TypedOutputEvent<std::tuple<M600Action::M600Action>> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "fcMissionWPAction"; };
  	
  	// User provided implementations
  	bool compare(dji_sdk::MissionWpAction::Request* message) override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class NavSucceededOUT : public TypedOutputEvent<bool> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "NavSucceeded"; };
  	
  	// User provided implementations
  	bool compare(uav_msgs::SpecialMovementActionResult::ConstPtr message) override;
};
class navCommandIN : public TypedInputEvent<NavCommand::NavCommand,uav_msgs::SpecialMovementActionGoal> {
  public:
  	// Inherit constructors
  	using TypedInputEvent::TypedInputEvent;
  	
	// Name
  	std::string getName() const override { return "navCommand"; };
  	
  	// Constructs ActionGoal message to be published
  	uav_msgs::SpecialMovementActionGoal getROSMessage() override {
  		uav_msgs::SpecialMovementActionGoal msg;
  		msg.goal = getROSGoalMessage();
  		return msg;
  	};
  	
  	// User provided implementations
  	uav_msgs::SpecialMovementGoal getROSGoalMessage();
  	
  	// Handles the acceptance of the goal
  	bool compare(actionlib_msgs::GoalStatus::ConstPtr message) override;
};

#endif /* _H_EVENTS */
