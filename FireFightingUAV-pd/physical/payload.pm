import physmod::math::* import physmod::joints::Revolute import physmod::sensors::Camera
pmodel Payload {
	const PAYLOAD_POLE_RADIUS : real = 1cm
	const ATTACHMENT_LENGTH : real = 32.4cm
	const dZ : real = 3.389 cm
	const dX : real = 7.5 cm

	local link Pole {
		def {
			inertial information {
					mass 0.500kg //baseline
					   + 0.0756kg // RealSense D435i weight per datasheet
					   + 1kg // Water tank?
							
				}
		}
		local body Pole {
			def {
				cylinder ( radius = PAYLOAD_POLE_RADIUS , length = 100.0cm )
				
			}
//			pose {
//				x = 0
//				y = 100/2
//				z = -dZ//-PAYLOAD_POLE_RADIUS
//				roll = 0
//				pitch = -PI/2
//				yaw = PI/2
//			}
		}
		pose {
				x = 0
				y = 100cm/2
				z = -dZ //-PAYLOAD_POLE_RADIUS
				roll = 0
				pitch = -PI/2
				yaw = PI/2
			}
	local body LeftPole {
			def {
				cylinder ( radius = 0.5cm , length = 100.0cm )
//				inertial information {
//					mass 0.100kg
//				}
			}
		pose {
				x = - 1.6106cm
				y = - 1.5cm
				z = 0
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body RightPole {
			def {
				cylinder ( radius = 0.5cm , length = 100.0cm )
//				inertial information {
//					mass 0.100kg
//				}
			}
		pose {
				x = - 1.6106cm
				y = 1.5cm
				z = 0
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
	local body GimbalMount {
			def {
				box ( length = 3.475cm , width = 5.9cm , height = 4cm )
//				inertial information {
//					mass 0.100kg
//				}
			}
			pose {
				x = - 1.6106cm - (3.475cm/2)+0.97cm
				y = 0.0
				z = -50cm+(4cm/2)
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
	
	jref GimballPan = Revolute {
			// Initially at 90 degrees
			flexibly connected to GimballPitchBase
			pose {
				x = - 1.6106cm - ( 3.475cm / 2 ) + 0.97cm
				y = 0
				z = - 50cm + 2cm
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		aref PanMotor = ServoMotor { relation GimballPan.tau == PanMotor.T }
			annotation Revolute {
				axis = Axis {
					xyz = (| 1, 0 , 0 |)
				}
			}
		instantiation AXIS = (|1,0,0|)
		}
	local body WaterTank {
			def {
				box ( length = 32.4cm , width = 10cm , height = 4cm )
//				inertial information {
//					mass 4kg
//				}
			}
			pose {
				x = - 4cm
				y = 0
				z = 32.4cm
				roll = 0.0
				pitch = PI / 2
				yaw = 0
			}
		}
	sref RealSense = DepthCamera
		sref ThermalCamera = ThermalCamera
	aref GCS = Comms
	sref RgbCamera = ColourCamera
	}
	local link Attachment {
		def {
			inertial information {
					mass 0.100kg
				}
		}
		pose {
			x = 0
			y = a*ATTACHMENT_LENGTH
			z = -dZ +8.23cm/2*sin(0.42440516)
			roll = 0
			pitch = 0
			yaw = 0
		}
		local body Left {
			def {
				cylinder ( 
					radius = 1.2cm / ( 2 * PI ) , 
					length = 8.23cm// FIXME: sqrt(dZ*dZ+dX*dX) doesn't work 
				)
				
			}
			pose {
				x = -dX/2 //- 8.23/2*cos(24.322968397)
				y = 0
				z = 0 //8.23/2*sin(24.322968397)
				roll = 0
				pitch = PI/2+0.42440516 // FIXME: atan ( dZ / dX ) doesn't work
				yaw = 0
			}
		}
		local body Right {
			def {
				cylinder ( 
					radius = 1.2cm / ( 2 * PI )  , 
					length = 8.23 cm// FIXME: sqrt(dZ*dZ+dX*dX) doesn't work 
				)
//				inertial information {
//					mass 0.100kg
//				}
			}
			pose {
				x = dX/2 //8.23/2*cos(24.322968397)
				y = 0
				z = 0 //8.23/2*sin(24.322968397)
				roll = 0
				pitch = PI/2+0.42440516 // FIXME: atan ( dZ / dX ) doesn't work
				yaw = PI
			}
		}
		fixed to Pole
		index a : <0,1>
	}
	local link GimballPitchBase {
		
		def {
			inertial information {
				mass 0.037kg/2 // Mini Pan-Tilt Kit has 37g of weight, here we split this into two links as 2-DOF
			}
		}
		local body Base {
			def {
				box ( length = 2cm , width = 2cm , height = 1cm )
//				inertial information {
//					mass 0.100kg
//				}
			}
			pose {
				x = 0
				y = 0
				z = 0 //- 1.5cm
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref GimballPitch = Revolute {
			// Initially at 90 degrees
			flexibly connected to Dispenser 
			aref PitchMotor = ServoMotor  { relation GimballPitch.tau == PitchMotor.T }
			annotation Revolute {
				axis = Axis {
					xyz = (| 1 , 0 , 0 |)
				}
			}
			instantiation AXIS = (|1,0,0|)
		} 
		pose {
			x = 0
			y = 100cm-2cm
			z = - ( 5.9cm / 2 ) - dZ -1.5cm
			roll = 0.0
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link Dispenser {
		def {
			inertial information {
				mass 0.037kg/2 // Mini Pan-Tilt Kit has 37g of weight, here we split this into two links as 2-DOF
			}
		}
	local body DispenserBase {
			def {
				box ( length = 2cm , width = 2cm , height = 1cm )
//				inertial information {
//					mass 0.100kg
//				}
			}
			pose {
				x = 0
				y = 0
				z = - 2cm
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		pose {
			x = 0.0
			y = 100cm-2cm
			z = - ( 5.9cm / 2 ) -dZ -2cm
			roll = 0.0
			pitch = 0.0
			yaw = 0.0
		}
	aref Nozzle = Nozzle
	}
}

