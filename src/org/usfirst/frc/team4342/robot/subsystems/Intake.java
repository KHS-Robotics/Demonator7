package org.usfirst.frc.team4342.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase 
{
	
	private boolean intaking;
	private boolean releasing;
	private TalonSRX motor;
	
	public Intake(TalonSRX motor)
	{
		this.motor = motor;
	}

	public void enable()
	{
		if(intaking)
			return;
		intaking = true;
		releasing = false;
		
		motor.set(ControlMode.PercentOutput, 1);
	}
	
	public void release()
	{
		if(releasing)
			return;
		intaking = false;
		releasing = true;
		
		motor.set(ControlMode.PercentOutput, -1);
	}
	
	public void disable()
	{
		if(intaking || releasing)
			return;
		intaking = releasing = false;
		
		motor.set(ControlMode.PercentOutput, 0);
	}
}
