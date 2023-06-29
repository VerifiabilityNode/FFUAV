pmodel UAV {
	part drone = M600 {
		pose {
			x = 0 
			y = 0
			z = 0
			roll = 0
			pitch = 0
			yaw = 0
		} 
		link MainFrame fixed to Attachment in payload 
		instantiation hubThickness = hubThickness
		instantiation attach_main_frame_length = attach_main_frame_length
		instantiation attach_main_frame_radius = attach_main_frame_radius
		instantiation attachment_height = attachment_height_uav
	}
	part payload = Payload {
		pose {
			x = 0
			y = -attach_main_frame_length/2 // half of MAIN_FRAME_LENGTH in attachment_base p-model. FIXME: pass these as parameters
			z = (-hubThickness/2)-attachment_height_uav-attach_main_frame_radius // (-baseThickness/2)-(attachment_length/2)- FIXME: pass these as parameters 
			roll = 0
			pitch = 0
			yaw = 0
		}
	}
	const hubThickness : real = 4.67cm
	const attach_main_frame_length : real = 32.4cm
	const attach_main_frame_radius : real = 0.6cm
	const attachment_height_uav : real = 6.28cm
}
