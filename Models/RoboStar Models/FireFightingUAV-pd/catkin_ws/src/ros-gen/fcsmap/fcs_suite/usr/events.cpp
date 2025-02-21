#include "event.hpp"
#include "events.hpp"
#include "executor.hpp"
#include "traceexecutor.hpp"
#include "types.hpp"
#include "primitivetypes.hpp"

bool fcActivationRetIN::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};

bool fcActivationRetIN::compare(dji_sdk::Activation::Response* message) {
	// To be completed.
	message->result = value;
	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcActivationRetIN's response.");
	#endif
	return true;
};
// bool uavStatusOUT::compare(uav_msgs::SpecialMovementActionResult::ConstPtr message) {
// 	// To be completed.
// 	if (message->result.done) {
// 		// At location, therefore we only match if RC event is AircraftStatus::AtWaypoint
// 		AircraftStatus::AircraftStatus status = AircraftStatus::AtWaypoint{};
// 		return value == status;
// 	} else {
// 		// Otherwise it's an error regarding waypoint setting
// 		AircraftStatus::AircraftStatus status = AircraftStatus::AtWaypointError{};
// 		return value == status;
// 	}
// };
bool fcMissionWPAction::compare(dji_sdk::MissionWpAction::Request* message) {
	// To be completed.
	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcMissionWPAction's request.");
	#endif

	M600Action::M600Action action;
	
	switch (message->action) {
		case dji_sdk::MissionWpAction::Request::ACTION_START:
			action = M600Action::Start{};
			return std::get<0>(value) == action;
		case dji_sdk::MissionWpAction::Request::ACTION_STOP:
			action = M600Action::Stop{};
			return std::get<0>(value) == action;
		case dji_sdk::MissionWpAction::Request::ACTION_PAUSE:
			action = M600Action::Pause{};
			return std::get<0>(value) == action;
		case dji_sdk::MissionWpAction::Request::ACTION_RESUME:
			action = M600Action::Resume{};
			return std::get<0>(value) == action;
		default:
			return false;
	}
};
bool fcMissionWPAction::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
bool batteryStateInIN::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
sensor_msgs::BatteryState batteryStateInIN::getROSMessage() {
	// To be completed.
	sensor_msgs::BatteryState msg;
	msg.percentage = value.percentage;
	msg.present = true;
	return msg;
};
bool fcMissionWPUploadRetIN::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
bool fcMissionWPUploadRetIN::compare(dji_sdk::MissionWpUpload::Response* message) {
	// To be completed.
	message->result = value;
	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcMissionWPUploadRetIN's response.");
	#endif
	return true;
};
bool fcControlAuthRetIN::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
bool fcControlAuthRetIN::compare(dji_sdk::SDKControlAuthority::Response* message) {
	// To be completed.
	message->result = value;
	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcControlAuthRetIN's response.");
	#endif
	return true;
};
bool fcActivation::compare(dji_sdk::Activation::Request* message) {
	// To be completed.
	// Nothing is passed as data in the request for activation, therefore always return true.
	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcActivation's request.");
	#endif
	return true;
};
bool fcActivation::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcActivation's accepted in.");
	#endif
	return true;
};
bool fcDroneTaskControl::compare(dji_sdk::DroneTaskControl::Request* message) {
	// To be completed.
	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcDroneTaskControl's request.");
	#endif

	M600Task::M600Task task;
	
	switch (message->task) {
		case dji_sdk::DroneTaskControl::Request::TASK_GOHOME:
			task = M600Task::GoHome{};
			return std::get<0>(value) == task;
		case dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF:
			task = M600Task::TakeOff{};
			return std::get<0>(value) == task;
		case dji_sdk::DroneTaskControl::Request::TASK_LAND:
			task = M600Task::Land{};
			return std::get<0>(value) == task;
		default:
			return false;
	}
};
bool fcDroneTaskControl::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
bool fcMissionWPActionRetIN::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
bool fcMissionWPActionRetIN::compare(dji_sdk::MissionWpAction::Response* message) {
	// To be completed.
	message->result = value;
	return true;
};
bool fcMissionWPUpload::compare(dji_sdk::MissionWpUpload::Request* message) {

	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcMissionWPUpload event.");
	#endif
	// To be completed.
	// Need to compare message->waypoint_task->mission_waypoint (MissionWaypoint[] vector)
	// if (message->waypoint_task != nullptr) {
	// 	if (message->waypoint_task.mission_waypoint != nullptr) {
			std::vector<dji_sdk::MissionWaypoint> mwps = message->waypoint_task.mission_waypoint;
			auto wps = mwps.begin();
			auto pos = std::get<0>(value).begin();
			
			while(wps != mwps.end() || pos != std::get<0>(value).end())
			{
				if(wps != mwps.end() && pos != std::get<0>(value).end())
				{
					if ((*wps).latitude == (*pos).getLatitude()
						&& (*wps).longitude == (*pos).getLongitude()
						&& (*wps).altitude == (*pos).getAltitude()) {
						++wps;
						++pos;
					} else {
						#ifdef DEBUG
							ROS_INFO_STREAM("Compare for fcMissionWPUpload event: wps do not match.");
						#endif
						return false;
					}
				} else { // Different length
					#ifdef DEBUG
						ROS_INFO_STREAM("Compare for fcMissionWPUpload event: wps are of different length.");
					#endif
					return false;
				}
			}
			#ifdef DEBUG
				ROS_INFO_STREAM("Compare for fcMissionWPUpload event: successful.");
			#endif
			return true; // Same length and equal contents, if any.
	// 	} else {
	// 		return false;
	// 	}
	// } else {
	// 	return false;
	// }
};

bool compareMissionWaypoint2Position(dji_sdk::MissionWaypoint wp, Position pos) {
	return pos.getLatitude() == wp.latitude && 
			pos.getLongitude() == wp.longitude &&
			pos.getAltitude() == wp.altitude;
};

bool fcMissionWPUpload::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
bool heightAboveTakeOffIN::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
std_msgs::Float32 heightAboveTakeOffIN::getROSMessage() {
	// To be completed.
	std_msgs::Float32 msg;
	msg.data = static_cast<float>(value);
	return msg;
};
bool navCommandIN::compare(actionlib_msgs::GoalStatus::ConstPtr message) {
	// To be completed.
	return true;
};
uav_msgs::SpecialMovementGoal navCommandIN::getROSGoalMessage() {
	// To be completed.
	uav_msgs::SpecialMovementGoal goal;
	uav_msgs::SpecialMovement movement;

	if (std::holds_alternative<NavCommand::TakeOff>(value)) {
		movement.data = uav_msgs::SpecialMovement::TAKE_OFF;
	} else if (std::holds_alternative<NavCommand::GoHome>(value)) {
		movement.data = uav_msgs::SpecialMovement::GO_HOME;
	} else if (std::holds_alternative<NavCommand::Land>(value)) {
		movement.data = uav_msgs::SpecialMovement::LAND;
	}
	goal.movement = movement;
	return goal;
};
bool batteryStateOutOUT::compare(uav_msgs::BatteryPercentage::ConstPtr message) {
	// To be completed.
	return value.percentage == message->percentage;
};
bool fcTaskRetIN::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
bool fcTaskRetIN::compare(dji_sdk::DroneTaskControl::Response* message) {
	// To be completed.
	message->result = value;
	#ifdef DEBUG
		ROS_INFO_STREAM("Compare for fcActivationRetIN's response.");
	#endif
	return true;
};
bool fcControlAuth::compare(dji_sdk::SDKControlAuthority::Request* message) {
	// To be completed.
	// There is no parameter in RoboChart, but here we ensure any requests we are comparing
	// are for request of control.
	return message->control_enable == dji_sdk::SDKControlAuthority::Request::REQUEST_CONTROL;
};
bool fcControlAuth::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
bool gpsPositionIN::compare(std_msgs::Time::ConstPtr message) {
	// To be completed.
	return true;
};
sensor_msgs::NavSatFix gpsPositionIN::getROSMessage() {
	// To be completed.
	sensor_msgs::NavSatStatus navstatus;
	navstatus.status = sensor_msgs::NavSatStatus::STATUS_FIX;
	navstatus.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

	sensor_msgs::NavSatFix navfix;
	navfix.status = navstatus;

	navfix.latitude = value.getLatitude();
	navfix.longitude = value.getLongitude();
	navfix.altitude = value.getAltitude();

	// no co-variance
	return navfix;
};
bool waypointIN::compare(actionlib_msgs::GoalStatus::ConstPtr message) {
	// To be completed.
	return true;
};
uav_msgs::FlyToWPGoal waypointIN::getROSGoalMessage() {
	// To be completed.
	uav_msgs::FlyToWPGoal goal;
	uav_msgs::GpsLocationWithPrecision gps;
	sensor_msgs::NavSatFix location;
	
	// Switch on cases.
	location.latitude = value.getLatitude();
	location.longitude = value.getLongitude();
	location.altitude = value.getAltitude();
	gps.location = location;
	goal.goal = gps;
	return goal;
};

bool WPSucceededOUT::compare(uav_msgs::FlyToWPActionResult::ConstPtr message) {
	return value == message->result.in_location;
};

bool NavSucceededOUT::compare(uav_msgs::SpecialMovementActionResult::ConstPtr message) {

	return value == message->result.done;

	// if (message->result.done) {
	// 	// At location, therefore we only match if RC event is AircraftStatus::AtWaypoint
	// 	AircraftStatus::AircraftStatus status = AircraftStatus::AtWaypoint{};
		
	// } else {
	// 	// Otherwise it's an error regarding waypoint setting
	// 	AircraftStatus::AircraftStatus status = AircraftStatus::AtWaypointError{};
	// 	return value == status;
	// }
};