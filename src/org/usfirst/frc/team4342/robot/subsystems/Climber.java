package org.usfirst.frc.team4342.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Climber subsystem to climb the tower
 */
public class Climber extends SubsystemBase 
{
	private boolean enabled;
	private TalonSRX motor;
	
	/**
	 * Creates a new <code>Climber</code> subsystem
	 * @param motor the motor to control the winch
	 */
	public Climber(TalonSRX motor)
	{
		this.motor = motor;
	}
	
	/**
	 * Enables the winch
	 */
	public void enable()
	{
		if(enabled)
			return;
		enabled = true;
		
		motor.set(ControlMode.PercentOutput, 1);
	}
	
	/**
	 * Disables the winch
	 */
	public void disable()
	{
		if(!enabled)
			return;
		enabled = false;
		
		motor.set(ControlMode.PercentOutput, 0);
	}
}