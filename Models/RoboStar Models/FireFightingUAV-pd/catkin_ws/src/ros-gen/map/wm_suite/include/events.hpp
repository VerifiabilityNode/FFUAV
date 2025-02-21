#ifndef _H_EVENTS
#define _H_EVENTS

#include "event.hpp"
#include "eventio.hpp"
#include "inputadapter.hpp"
#include "types.hpp"
#include "primitivetypes.hpp"

class sprayInIN : public TypedInputEvent<PumpCommand::PumpCommand,uav_msgs::EnableWaterMonitor::Request> {
  public:
  	// Inherit constructors
  	using TypedInputEvent::TypedInputEvent;
  	
	// Name
  	std::string getName() const override { return "sprayIn"; };
  	
  	// User provided implementations
  	uav_msgs::EnableWaterMonitor::Request getROSMessage() override;
  	bool compare(std_msgs::Time::ConstPtr message) override;
};
class waterStatusOutOUT : public TypedOutputEvent<WaterStatus::WaterStatus> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "waterStatusOut"; };
  	
  	// User provided implementations
  	bool compare(uav_msgs::WaterStatus::ConstPtr message) override;
};

#endif /* _H_EVENTS */
