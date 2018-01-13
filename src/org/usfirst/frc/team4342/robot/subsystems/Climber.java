package org.usfirst.frc.team4342.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Climber extends SubsystemBase 
{

	private boolean enabled;
	private TalonSRX motor;
	
	public Climber(TalonSRX motor)
	{
		this.motor = motor;
	}
	
	public void enable()
	{
		if(enabled)
			return;
		enabled = true;
		
		motor.set(ControlMode.PercentOutput, 1);
	}
	
	public void disable()
	{
		if(!enabled)
			return;
		enabled = false;
		
		motor.set(ControlMode.PercentOutput, 0);
	}
}