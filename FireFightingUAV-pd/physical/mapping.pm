import physmod::math::*
import sequence_toolkit::*
import physmod::sensors::*

enumeration OPosition {
	none
	position(Position)
}

map Mapping dmodel FirefightingUAV to pmodel UAV
{	
	// Linear and angular speed for the drone when moving between waypoints
	const angularSpeed : real
	const take_off_speed : real = 0.5 // 1.2m~1.5m above ground 
	const cruising_speed : real = 2.5 // moving between waypoints -- 2.5m/s for speed of movement
	const landing_speed : real = 1.0 // when landing initially it's 1m/s
	const landing_speed_close_to_ground : real = 0.5  // when distance 1m to actual ground: 0.5m/s
		
	// Desired WayPoint as set, for example, by fcWaypointCall. Initially it is set as none.
	var desiredWP : OPosition = OPosition::none
	
	// Desired relative positioning, when set via fcPositionAndYawCtrl.
	var relative : vector(real,4)
	
	// Keep track of whether in the air.
	var onair : boolean = false
	
	// This must be set as the initial position (LLA) when the drone takes off.
	var origin : vector(real,3) = [|0,0,0|]
	
	// Here we record the home Longitude/Latitude.
	// See: https://developer.dji.com/onboard-api-reference/structDJI_1_1OSDK_1_1Telemetry_1_1HomeLocationData.html
	//
	// According to: https://developer.dji.com/mobile-sdk/documentation/introduction/component-guide-flightController.html
	// "When automatically going home, the aircraft will rise to a minimum altitude, fly to the home location 
	//  (home point) using GPS positioning, then land."
	//
	// Question: what happens if the drone takes off/lands from slanted surface?
	// ANSWER: It cannot take off from slanted surface.
	var home : vector(real,3)
		
	/******************************************************************************************
	 * Waypoint navigation
	 ******************************************************************************************/
	 
	// fcWaypointCall(pos: Position)
	operation fcWaypointCall { 		
		var lla_fcWPC:vector(real,3)  = UAV::drone::Hub::GPS.oposition
		var ned_fcWPC:vector(real,3)  = LLA2NED(lla_fcWPC,origin)
		var time_ds : int = ceiling(distance(ned_fcWPC,LLA2NED(Position2Vector(pos),origin))/cruising_speed)
		
		action  desiredWP=OPosition::position(pos);
				wait([0,time_ds]); //where t is defined by the time needed for straight line movement between current position and target.
				//
				// ASSUMES: roll/pitch assumed to be 0 wrt. NED frame 
				pose = (|LLA2NED(Position2Vector(pos),origin)[1], // X
						 LLA2NED(Position2Vector(pos),origin)[2], // Y
						 LLA2NED(Position2Vector(pos),origin)[3], // Z
						 0,0,deg2rad(pos.heading)|) // Yaw
	}
	input event fcWaypointFinishedMission {
		equation desiredWP != OPosition::none /\
					(the 	i_wp:Position | desiredWP==OPosition::position(i_wp)
							@ 
							(UAV::drone::Hub::GPS.oposition)[1] == i_wp.lat
						/\	(UAV::drone::Hub::GPS.oposition)[2] == i_wp.lon
						/\  (UAV::drone::Hub::GPS.oposition)[3] == i_wp.alt
						/\  (UAV::drone::Hub::IMU.oorient)[3] == i_wp.heading
					)
	}
	
	/******************************************************************************************
	 * Take off
	 ******************************************************************************************/
	 
	// fcTakeoffCall()
	operation fcTakeoffCall {
		
		var lla_fcT:vector(real,3)  = UAV::drone::Hub::GPS.oposition
		var ned_fcT:vector(real,3)  = LLA2NED(lla_fcWPC,origin)
		var time_fcT : int = ceiling(distance(ned_fcT,[|0,0,-1.2|])*take_off_speed)
		
		// Take off to 1.2m above ground.
		//
		// QUESTION: can this be called multiple times? What is the behaviour?
		// ANSWER: yes, it could, but no behaviour if already in the air.
		//
		// RATIONALE: 1. 'onair' keeps track of whether fcTakeoffCall has been called
		// 			  2. (a) if not already 'onair', then we set the origin as the current
		//				 location given by GPS, then set the pose to 1.2m above. Home
		//				 position is on the ground.
		//
		// 				 (b) if already 'onair', the behaviour is skip, nothing changes.
		//
		action if onair then 
					skip 
			   else onair=true;
			   		origin=UAV::drone::Hub::GPS.oposition;
			   		wait([0,time_fcT]);
			   		pose = (|ned_fcT[1],ned_fcT[2],ned_fcT[3]-1.2,0,0,0|);
			   		home = [|ned_fcT[1],ned_fcT[2],ned_fcT[3]|]
			   end
	}
	input event fcTakeoffRet {		
		// Event is available if height is 1.2m above ground.
		// Sufficient to check Z coordinate (in NED frame) is greater than -1.2
		equation LLA2NED(UAV::drone::Hub::GPS.oposition,origin)[3] >= -1.2
	}
	
	/******************************************************************************************
	 * GoHome
	 ******************************************************************************************/
	 
	// fcGoHomeCall()
	operation fcGoHomeCall {
		var lla_fcGH:vector(real,3)  = UAV::drone::Hub::GPS.oposition
		var ned_fcGH:vector(real,3)  = LLA2NED(lla_fcGH,origin)
		var time_GH : int = ceiling(distance(ned_fcGH,(|home[1],home[2],home[3]-1.2|))/cruising_speed)
						  + ceiling(distance((|home[1],home[2],home[3]-1.2|),(|home[1],home[2],home[3]|))/landing_speed)
		
		action 	wait([0,time_GH]);
				// QUESTION: do we keep current pitch/yaw the same?
				// RATIONALE: for now we set RPY to all 0 (FRU wrt. NED)
				pose = (|home[1],home[2],home[3],0,0,0|)
	}
	input event fcGoHomeRet {
		equation LLA2NED(UAV::drone::Hub::GPS.oposition,origin) == home
	}
	
	/******************************************************************************************
	 * Relative positioning in local FRU (Front-Rear-Up) frame of reference
	 ******************************************************************************************/

	// fcMoveRelativeCall(movement: RelativeMotion)
	// This assumes offsets in FRU.
	operation fcMoveRelativeCall { 
		
		var lla_fcMRC : vector(real,3)  = UAV::drone::Hub::GPS.oposition
		var ned_fcMRC : vector(real,3)  = LLA2NED(lla_fcMRC,origin)
		var movement_NED : vector(real,3) = FRU2NED([|movement.dx,movement.dy,movement.dz|],UAV::drone::Hub::IMU.oorient)
		var time_fcMRC : int = ceiling(distance(ned_fcGH,(|home[1],home[2],home[3]-1.2|))/cruising_speed)
		
		// Given current pose, obtained from IMU+GPS, calculate new pose
		action wait([0,time_fcMRC]);
			   relative[1] = ned_fcMRC[1]+movement_NED[1];
			   relative[2] = ned_fcMRC[2]+movement_NED[2];
			   relative[3] = ned_fcMRC[2]+movement_NED[3];
			   relative[4] = movement.dyaw 
	}
	input event fcMoveRelativeRet {
		equation 	LLA2NED(UAV::drone::Hub::GPS.oposition,origin)[1] == relative[1]
				/\	LLA2NED(UAV::drone::Hub::GPS.oposition,origin)[2] == relative[2]
				/\	LLA2NED(UAV::drone::Hub::GPS.oposition,origin)[3] == relative[3]
				/\  (UAV::drone::Hub::IMU.oorient)[3] == relative[4]
	}
	
	/******************************************************************************************
	 * Landing
	 ******************************************************************************************/
	 
	// fcLandCall()
	operation fcLandCall {
		
		var lla_fcLC:vector(real,3)  = UAV::drone::Hub::GPS.oposition
		var ned_fcLC:vector(real,3)  = LLA2NED(lla_fcLC,origin)
		var time_LC : int = ceiling(distance(ned_fcLC,(|ned_fcLC[1],ned_fcLC[2],0|))/landing_speed)
		
		// QUESTION: How does the drone know it has landed?
		// ANSWER: This is based on 'height relative to ground' via observing changes in height.
		
		// But how will we appropriately model this? In the actual system
		// landing is performed progressively, first by descending at 1m/s and
		// when closer to the ground at 0.5m/s, until height no longer changes.
		// This would require a feedback loop system: can it be specified as a DAE system here?
		
		// RATIONALE: For now we set the Z-axis (NED) at 0 and assume that the local NED is planar.
		action  wait([0,time_LC]);
				pose[3]=0
	}
	
	input event fcLandRet {
		// If height relative to ground is less than or equal to 0.
		// COMMENT: In reality this is a control loop.
		equation LLA2NED(UAV::drone::Hub::GPS.oposition,origin)[3]<=0
	}

	/******************************************************************************************
	 * Gimbal commands
	 ******************************************************************************************/
	 
	// gimbalWritePan(angle: int)
	// COMMENT: angle is provided in degrees 0-180
	operation gimbalWritePan { 
		action UAV::payload::Pole::GimballPan::PanMotor.dangle = deg2rad(angle)
	}
	
	// gimbalWriteTilt(angle: int)
	// COMMENT: angle is provided in degrees 0-180
	operation gimbalWriteTilt { 
		action UAV::payload::GimballPitchBase::GimballPitch::PitchMotor.dangle = deg2rad(angle)
	}
	
	/******************************************************************************************
	 * mlx inputs and commands
	 ******************************************************************************************/
	 	
	input event mlxHeatmap?hm {
		// TODO: Investigate conversion from real->int in the output of camera sensor.
		//equation hm==Frame(|contents=UAV::payload::Pole::ThermalCamera.image|)
	}

	/******************************************************************************************
	 * Pump commands
	 ******************************************************************************************/
	 
	// pumpOff()
	operation pumpOff { 
		equation UAV::payload::Dispenser::Nozzle.V == 0
	}
	// pumpOn()
	operation pumpOn { 
		// TODO/QUESTION: 4L/minute troughput for the pump yet to be reflected
		equation UAV::payload::Dispenser::Nozzle.V > 0
	}
	
	/******************************************************************************************
	 * Battery reading
	 ******************************************************************************************/	
	input event batteryInfo?d {
		equation d == Battery(|percentage=UAV::drone::Hub::CurrentBattery.opercentage|)
	}
	
	/******************************************************************************************
	 * Control authority
	 ******************************************************************************************/
	input event fcControlAuth {
		// Initial delay up to 1minute before fcControlAuth is available
		equation t >= 60
	}
	
	/******************************************************************************************
	 * Mission status
	 ******************************************************************************************/
	 
	// missionStatus: MissionStatus, where MissionStatus is an enumeration
	output event missionStatus!e { 
		equation UAV::payload::Pole::GCS.inp == missionString(e) // Maps enumerated value to a string.
	}
	
	/******************************************************************************************
	 * RealSense camera inputs
	 ******************************************************************************************/
	input event rsDepthFrame?rsFrame {
		//equation rsFrame==Frame(|contents=UAV::payload::Pole::RealSense.image|)
	}
	
	input event rsRgbFrame?rgbFrame {
		//equation rgbFrame==RgbFrame(|contents=Colour2Rgb(UAV::payload::Pole::RgbCamera.image)|)
	}
}

// Additional notes and remarks.
//
// 1. We do not need to worry about the change in mass of the water tank.
//
//    It is mounted so that the centre of gravity doesn't change much. Besides,
//	  It may give slightly longer battery in the region of a few minutes, but
//	  not more than that. So we can avoid modelling this for now.
	
/******************************************************************************************
 * Auxiliar functions
 ******************************************************************************************/
 
function missionString(e:MissionStatus):string {
	postcondition 
			e==MissionStatus::Connected 	/\ result=="Connected"
//		\/ 	e==MissionStatus::Initialising 	/\ result=="Initialising"
//		\/ 	e==MissionStatus::Initialised 	/\ result=="Initialised"
		\/ 	e==MissionStatus::TakingOff 	/\ result=="Taking off"
		\/ 	e==MissionStatus::FlyingToWall 	/\ result=="Flying to Wall"
		\/ 	e==MissionStatus::Searching 	/\ result=="Searching"
		\/ 	e==MissionStatus::TrackingFire 	/\ result=="Tracking Fire"
		\/ 	e==MissionStatus::Spraying 		/\ result=="Spraying"
		\/ 	e==MissionStatus::SearchComplete/\ result=="Search complete"
		\/ 	e==MissionStatus::GoingHome 	/\ result=="Going Home"
		\/ 	e==MissionStatus::Landing 		/\ result=="Landing"
		\/ 	e==MissionStatus::Disarming 	/\ result=="Disarming"
		\/ 	e==MissionStatus::ConnectTimeout/\ result=="Connection Timeout"
//		\/ 	e==MissionStatus::InitError 	/\ result=="Initialisation Error"
		\/ 	e==MissionStatus::BatteryError 	/\ result=="Battery Error"
		\/ 	e==MissionStatus::WaterError 	/\ result=="Water Error"
}

// Given an LLA (Latitude/Longitude/Altitude) coordinate this function 
// yields a vector with the position in the ECEF coordinate system using
// the World Geodetic System 1984 (WGS84). See https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf
function LLA2ECEF(i:vector(real,3)):vector(real,3) {
	postcondition 
		let a==6378137,
			f==1/(298.257223563),
			b==(a*(1-f)),
			e==sqrt((a cat 2-b cat 2)/a cat 2),
			N==a/(sqrt(1-(e cat 2*sin(i[1]) cat 2))),
			X==(N+i[3])*cos(i[1])*cos(i[2]),
			Y==(N+i[3])*cos(i[1])*sin(i[2]),
			Z==((b cat 2/a cat 2)*N+i[3])*sin(i[1]) @ result==[|X,Y,Z|]
}

// From ECEF position 'pe' to NED frame of reference assuming position of
// the origin of local NED frame is at llaOrigin.
//
// Equations adopted from p.32 of "Umanned Rotorcraft Systems".
function ECEF2NED(pe:vector(real,3),llaOrigin:vector(real,3)):vector(real,3) {
	postcondition
		let
			peref == LLA2ECEF(llaOrigin),
			lon == llaOrigin[1],
			lat == llaOrigin[2]
		@
		result==[| -sin(lon)*cos(lat), -sin(lon)*sin(lat), cos(lon);
				    -sin(llaOrigin[2]), cos(lon), 0;
				    -cos(lon)*cos(lat), -cos(lon)*sin(lat), -sin(lat) |]*(pe-peref)
}

function LLA2NED(i:vector(real,3),llaOrigin:vector(real,3)):vector(real,3) {
	postcondition
		result==ECEF2NED(LLA2ECEF(i),llaOrigin)
}

// Mapping from Position (LLA) to ECEF using WGS84.
function Position2ECEF(p:Position):vector(real,3) {
	postcondition result==LLA2ECEF([|p.lat,p.lon,p.alt|])
}

function Position2Vector(p:Position):vector(real,3) {
	postcondition result==[|p.lat,p.lon,p.alt|]
}

// Converts degrees to radians
function deg2rad(a:real):real {
	postcondition result==a*PI/180
}
function intdeg2rad(a:int):real {
	postcondition result==a*PI/180
}

// Straight-line distance between org and tgt.
function distance(org:vector(real,3),tgt:vector(real,3)):real {
	postcondition result==sqrt((tgt[1]-org[1]) cat 2+(tgt[2]-org[2]) cat 2+(tgt[3]-org[3]) cat 2)
}

function ceiling(p:real):int {
	postcondition result==min({ n:int | n >= p})
}

// Takes vector p (XYZ) and rotates it according to Roll/Pitch/Yaw Euler angles (rpy, radians)
function FRD2NED(p:vector(real,3),rpy:vector(real,3)):vector(real,3) {
	postcondition 
		let 
			ro==rpy[0],
			pi==rpy[1],
			ya==rpy[2]
		@
		result==[| cos(ro)*cos(pi)-sin(ya)*sin(ro)*sin(pi), -cos(ya)*sin(ro), cos(ro)*sin(pi)+cos(pi)*sin(ya)*sin(ro);
				   cos(ya)*sin(ro)-cos(ro)*sin(ya)*sin(pi), cos(pi)*cos(ro), sin(ro)*sin(pi)-cos(pi)*sin(ya)*cos(ro);
				   -cos(ya)*sin(pi), sin(ya), cos(ya)*cos(pi) |]*p
}

function FRU2NED(p:vector(real,3),rpy:vector(real,3)):vector(real,3) {
	postcondition result==FRD2NED([|p[1],p[2],-p[3]|],rpy)
}
 
function Colour2Rgb(cm:matrix(Colour,960,720)):matrix(Rgb,960,720) {
	postcondition 
		forall r:nat, c:nat | r <= 960 /\ c <= 720 @ 
			the cr:real, cg:real, cb:real | cm(r,c) == Colour::rgb(cr,cg,cb)
				@ result(r,c) == Rgb(|r=cr,g=cg,b=cb|)
}

//	// The GPS reading is a point within 'poseDelta' distance from the reading obtained
//	// from the IMU. ie. take the IMU reading and within 'poseDelta' radius sphere. 
//	input event gps?p {
//		const poseDelta : real
//		var rpos : vector(real,6)
//		var rr : real
//		
//		equation rpos in {vp:vector(real,6)|distance(vp,UAV::drone::Hub::GPS.oposition) <= poseDelta}
//		equation p == Position(|lat=rpos[1],lon=0,alt=0,heading=0|) 
//		
//	}


