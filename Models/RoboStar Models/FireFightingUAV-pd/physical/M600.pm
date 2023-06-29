import physmod::math::*
const PI: real = 3.1415 import physmod::joints::Revolute import physmod::actuators::LinearSpeedControlMotor import physmod::sensors::Camera //Shouldn't this be in a library?

pmodel M600 {
	
	const hubThickness : real //= 1.4*2 //cm
	
	local link Hub {
		def {
			inertial information {
					mass 9.6kg
				}
		}
		local body Hub {
		 	def {
		 		cylinder (
		 			radius = hubRadius, 
		 			length = hubThickness
		 		)
		 		
		 	}
		}
	fixed to arm in arm
//	jref LHinge = Revolute {
//			pose {
//				x = 0
//				y = 0
//				z = 0
//				roll = 0
//				pitch = 0
//				yaw = 0
//			}
//		}
//		jref RHinge = Revolute {
//			pose {
//				x = 0
//				y = 0
//				z = 0
//				roll = 0
//				pitch = 0
//				yaw = 0
//			}
//		}
//	local joint LHinge {
//			def {
//			}
//			pose {
//				x = 0
//				y = 0
//				z = 0
//				roll = 0
//				pitch = 0
//				yaw = 0
//			}
//		}
//		local joint RHinge {
//			def {
//			}
//			pose {
//				x = 0
//				y = 0
//				z = 0
//				roll = 0
//				pitch = 0
//				yaw = 0
//			}
//		}
//	jref LHinge = Revolute {
//			flexibly connected to gear in LeftLeg 
//			pose {
//				x = hubRadius
//				y = 0
//				z = -hubThickness/2
//				roll = 0
//				pitch = 0
//				yaw = 0
//			}
//			instantiation AXIS = (|0,1,0|) 
//		}
//	jref RHinge = Revolute {
//			flexibly connected to gear in RightLeg 
//			pose {
//				x = - hubRadius
//				y = 0
//				z = -hubThickness/2
//				roll = 0
//				pitch = 0
//				yaw = 0
//			}
//			instantiation AXIS = (|0,1,0|)
//		}
	sref IMU = IMU
	sref GPS = Positioning
	sref CurrentBattery = Battery
	aref Transmitter = Transmitter
	}
	const arms : nat = 6
	
	// distance of arm from origin
	//const armLength : real = 10.0
	
	// angle with respect to edge of Base
	const armZAngle : real = -8.0
	
	const propellerZAngle : real = 1.0
	
	const hubRadius : real = 16.812 cm
	
	part arm = RotorArm {
		pose {
			x = hubRadius*cos(a*(2*PI/arms)) //*(armLength*cos(armZAngle)) 
			y = hubRadius*sin(a*(2*PI/arms)) //*(armLength*cos(armZAngle))
			z = 0 //armLength*sin(armZAngle) 
			roll = 0
			pitch = armZAngle*PI/180 //armZAngle 
			yaw = a*(2*PI/arms)
		}		
		//instantiation arm::ARM_LENGTH = armLength
		index a : [1,arms]
	}
	
	// This is positioned under an arm.
	part LeftLeg = LandingGear {
		pose {
			x = hubRadius
			y = 0
			z = -hubThickness/2
			roll = 0
			pitch = -10*PI/180
			yaw = 0
		}
	link gear fixed to Hub
	}
	
	// This is positioned under an arm.
	part RightLeg = LandingGear {
		pose {
			x = -hubRadius
			y = 0
			z = -hubThickness/2
			roll = 0
			pitch = -10*PI/180
			yaw = PI
		}
	link gear fixed to Hub
	}
	local link MainFrame {
		def {
		}
		local body MainFrameBody {
			pose {
				x = f*attach_side_frame_distance/2
				y = 0
				z = 0
				roll = -PI/2
				pitch = 0.0
				yaw = 0.0
			}
			def {
				cylinder ( radius = attach_main_frame_radius , length = attach_main_frame_length )
			}
			index f : < -1,1>
		}
		local body SideFrameBody {
			def {
				box ( length = attach_side_frame_distance, width = attach_side_frame_width, height = attach_side_frame_height )
			}
			pose {
				x = 0
				y = f*attach_main_frame_length/2
				z = 0
				roll = 0
				pitch = 0
				yaw = 0
			}
			index f : < -1,1>
		}
		local body Pole {
			pose {
				x = hubRadius*cos(a*(2*PI/arms)) + (attachment*cos((a*(2*PI/arms)) + PI))
				y = hubRadius*sin(a*(2*PI/arms)) + (attachment*sin((a*(2*PI/arms)) + PI))
				z = (attachment_height/2) // Following unnecessary given containment: (-hubThickness/2)-(attachment_height/2)
				roll = 0
				pitch = 0
				yaw = 0
			}
			def {
				cylinder ( radius = attachment_pole_radius , length = 6.28cm )
			}
			index a : <1,2,4,5>
		}
		
		pose {
			x = 0
			y = 0
			z = ( - hubThickness / 2 ) - attachment_height
			roll = 0
			pitch = 0
			yaw = 0
		}
//		fixed to SideFrame
//		index f : < -1,1>
	fixed to Hub
	}
//	local link SideFrame {
//		def {
//		}
//		local body SideFrameBody {
//			def {
//				box ( length = attach_side_frame_distance, width = attach_side_frame_width, height = attach_side_frame_height )
//			}
//		}
//		pose {
//			x = 0
//			y = f*attach_main_frame_length/2
//			z = ( - hubThickness / 2 ) - attachment_height
//			roll = 0
//			pitch = 0
//			yaw = 0
//		}
//		index f : < -1,1>
//	}
	const attachment : real = 1.9004cm //-(attachment_pole_radius/2) //0.93
	const attachment_height : real //= 6.28 
	const attachment_pole_radius : real = 0.5 cm
	/* Constants governing the dimensions of the frame beneath the M600 */
	const attach_main_frame_length : real //= 32.4
	const attach_side_frame_distance : real = 14.92cm
	const attach_side_frame_height : real = 0.68cm
	const attach_side_frame_width : real = 0.76cm
	const attach_main_frame_radius : real
}

