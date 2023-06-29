pmodel LandingGear {
	
	// Parameters
	const LENGTH : real = 25.31 cm
	const FOOT_LENGTH : real = 11.25 cm
	
	local link gear {
		def {
		}
		local body leg {
			def {
				cylinder ( 
					radius = 1.25cm 
					, length = LENGTH
				)
			}
			pose {
				x = 0
				y = 0
				z = -LENGTH/2
				roll = 0
				pitch = -PI
				yaw = 0
			}
		}
		local body foot {
			def {
				cylinder ( 
				radius = 0.90cm 
				,length = FOOT_LENGTH )
			}
			pose {
				x = 0
				y = 0
				z = -LENGTH
				roll = PI/2
				pitch = -PI/2
				yaw = 0
			}
		}
		pose {
			x = 
			0
			y = 0
			z = 0
			roll = 0
			pitch = 0
			yaw = 0
		}
	}
}
