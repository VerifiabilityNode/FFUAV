
import relation_toolkit::*
import physmod::math::*
import physmod::sensors::*

// A very simplistic IMU that simply takes a pose as input and passes it along.
// The pose is in reference to an inertial reference frame. In the containment
// relation of how the IMU sensor is used we record its mounting position, but
// how can we use this information to write equations about it?
//
// There are some alternatives:
//
// 1. Use integration of accelerometer+gyroscope over time, assuming no drift
//	  for a 'perfect' positioning system. In reality, this is often corrected
//	  with some other kind of positioning input (eg. GPS) and Kalman filtering.
//
// 2. Fusion of sensor inputs, eg. integration over time of accelerometer+gyroscope
// 	  but also incorporation of GPS data. A GPS sensor would be defined as having
//	  as input polar coordinates? Note that there is also uncertainty in such a 
// 	  reading due to accuracy limitation of positioning systems.
//
sensor IMU {
	// Roll/Pitch/Yaw (ground frame NED to body frame FRD rotation)
	input iorient : vector ( real , 3 )
	output oorient : vector ( real , 3 )
	equation oorient == iorient 
}

sensor Positioning {
	// Latitude/longitude/altitude
	input iposition : vector ( real , 3 )
	output oposition : vector ( real , 3 )
	equation iposition == oposition 
}

// Idealised sensor for battery percentage.
sensor Battery {
	const initial_battery_level : real = 100
	const battery_decay_rate : real = -0.05 // Decay rate: 0.05%/s
	
	output opercentage : nat
	equation derivative(opercentage) == battery_decay_rate
	equation opercentage(0) == initial_battery_level
}

// Need further details as to how to capture this appropriately. It should output a
// depth map.
sensor DepthCamera {

	// Taken from RealSense D435i's self-reported parameters
	// Intrinsic of "Depth" / 640x480 / {Z16}
	const WIDTH: nat = 960, HEIGHT: nat = 720 // Resolution, max is: 1280x700
	const mx: nat, my: nat
	const CM: matrix(real,3,4) =
		[| 385.365936279297, 0, 317.542877197266, 0;
		   0, 385.365936279297, 243.07763671875, 0;
		   0, 0, 1, 0|]
	
	// Mapping from a position in the world to an observed distance (real)
	input world: Set(vector(real,3))
	
	// Depth map
	output image: matrix(real,960,720)
	
	local trans: vector(real,3) -> real
	local plane: vector(real,2) -> real
	
	//precondition: xyIsUnique(world)
		
	// hom3(op) makes a 3-vector into a 4-vector by adding a one as the last coordinate,
	// trans associates each of the world points, transformed via CM*hom3, to its temperature
	equation trans == {op: vector(real,3) | op in world @ (|CM*hom3(op), op[3]|)}
	
	// associates each of dehom2(CM*hom3(op)), where dehom2(x) takes a vector and drops its
	// third component, and associates it with trans(p), which ultimately associates 'p' with its temperature.
	equation plane == {p: vector(real,3) | p in dom(trans) @ (|dehom2(p), trans(p)|)}
	
	// for all px in [1,WIDTH] and py in [1,HEIGHT], the image at (px,py) corresponds to
	// that of plane at (mx*px,my*py). 
	equation forall px: nat, py: nat | 1 <= px /\ px <= WIDTH /\ 1 <= py /\ py <= HEIGHT @ 
		image(px,py) == plane(mx*px,my*py)
}

// This is a model of a thermal camera based on that of the Camera in the RoboSim library.
//
// For now the only notable difference is that there is no colour associated with the
// world, but rather a temperature (real).
//
sensor ThermalCamera {
	
	const WIDTH: nat = 960, HEIGHT: nat = 720 // Resolution
	const mx: nat, my: nat
	// Intrinsic characteristics of camera, including focal length
	const CM: matrix(real,3,4)
	
	// Input is a vision of the world from the camera's point of view,
	// that is, a mapping from 3D points to temperature.
	input world: vector(real,3) -> real
	
	// NOTE: Specify upscaling here, from 32x24 to 960x720.
	output image: matrix(real,960,720)
	
	local trans: vector(real,3) -> real
	local plane: vector(real,2) -> real
	// precondition: xyIsUnique(world)
		
	// hom3(op) makes a 3-vector into a 4-vector by adding a one as the last coordinate,
	// trans associates each of the world points, transformed via CM*hom3, to its temperature
	equation trans == {op: vector(real,3) | op in dom(world) @ (|CM*hom3(op), world(op)|)}
	
	// associates each of dehom2(CM*hom3(op)), where dehom2(x) takes a vector and drops its
	// third component, and associates it with trans(p), which ultimately associates 'p' with its temperature.
	equation plane == {p: vector(real,3) | p in dom(trans) @ (|dehom2(p), trans(p)|)}
	
	// for all px in [1,WIDTH] and py in [1,HEIGHT], the image at (px,py) corresponds to
	// that of plane at (mx*px,my*py). 
	equation forall px: nat, py: nat | 1 <= px /\ px <= WIDTH /\ 1 <= py /\ py <= HEIGHT @ 
		image(px,py) == plane(mx*px,my*py)
}

sensor ColourCamera {
	
	// Taken from RealSense D435i's self-reported parameters
	// Intrinsic of "Color" / 640x480 / {RGB8}
	const WIDTH: nat = 960, HEIGHT: nat = 720 // Resolution, max is 1920x1080
	const mx: nat, my: nat
	const CM: matrix(real,3,4) = 
		[|611.201416015625, 0, 321.567657470703, 0;
		  0, 611.634033203125, 247.377578735352, 0;
		  0, 0, 1, 0|]

	input world: vector(real,3) -> Colour
	output image: matrix(Colour,960,720)
	
	local trans: vector(real,3) -> Colour
	local plane: vector(real,2) -> Colour

	equation trans == {op: vector(real,3) | op in dom(world) @ (|CM*hom3(op), world(op)|)}
	equation plane == {p: vector(real,3) | p in dom(trans) @ (|dehom2(p), trans(p)|)}
	equation forall px: nat, py: nat | 1 <= px /\ px <= WIDTH /\ 1 <= py /\ py <= HEIGHT @ 
		image(px,py) == plane(mx*px,my*py)
	
}
