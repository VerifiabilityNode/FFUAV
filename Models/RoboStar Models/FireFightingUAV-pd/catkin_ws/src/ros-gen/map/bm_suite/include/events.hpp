#ifndef _H_EVENTS
#define _H_EVENTS

#include "event.hpp"
#include "eventio.hpp"
#include "inputadapter.hpp"
#include "types.hpp"
#include "primitivetypes.hpp"

class batteryStatusOUT : public TypedOutputEvent<BatteryStatus::BatteryStatus> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "batteryStatus"; };
  	
  	// User provided implementations
  	bool compare(uav_msgs::BatteryStatus::ConstPtr message) override;
};
class batteryStateIN : public TypedInputEvent<Battery,uav_msgs::BatteryPercentage> {
  public:
  	// Inherit constructors
  	using TypedInputEvent::TypedInputEvent;
  	
	// Name
  	std::string getName() const override { return "batteryState"; };
  	
  	// User provided implementations
  	uav_msgs::BatteryPercentage getROSMessage() override;
  	bool compare(uav_msgs::InputAccepted::ConstPtr message) override;
};

#endif /* _H_EVENTS */
