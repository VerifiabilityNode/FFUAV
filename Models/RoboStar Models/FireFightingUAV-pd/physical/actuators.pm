import physmod::math::*

// Pump/Nozzle
actuator Nozzle {
	
	// Receives a voltage and current. 
	input V:real, I:real
	local Q:real
	local Ve:real // Velocity
	
	
	// Constant pressure, max 10bar
	const Volume:real // 
	const P:real // Pressure
	const g:real // Gravity
	const h:real // Head height. Usually a variable, but in our case a constant because
				 // bag location is fixed.
	
	// Area of the nozzle section?
	const A:real
	
	// It produces a force on the water coming out of the nozzle.
	output F:real
	
	equation Ve==sqrt(2*g*h)
	equation Q == A*Ve
	equation P == F/A
}

actuator ServoMotor {
	
	input dangle: real 
	output T: real
	local Tm: real, Vemf: real, Tf: real 
	local V: real, i: real
	local theta: real, av: real, e: real
	
	const b: real, Ke: real, Kt: real
	const R: real, L: real
	const Kp: real, Ki: real, Kd: real
	
	equation av == derivative(theta)
	equation Tm == Kt*i
	equation Vemf == Ke*av
	equation Tf == b*av
	equation T == Tm - Tf
	equation V == i*R+L*derivative(i)+Vemf 
	equation e == dangle-theta //das - av
	equation V == Kp*e+Ki*integral(e,0,t)+Kd*derivative(e)
}

actuator Comms {
	input inp:string
	output oup:string
	equation oup==inp
}

actuator Transmitter {
	
	output batteryLevel:nat
	
	local blevel:real
	const initial_battery_level : real = 100
	const battery_decay_rate : real = -0.05 // Decay rate: 0.05%/s
	
	equation derivative(blevel) == battery_decay_rate
	equation blevel(0) == initial_battery_level
	equation batteryLevel == ceiling(blevel)
}