package org.usfirst.frc.team4342.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Intake subsystem to intake and release cubes
 */
public class Intake extends SubsystemBase 
{
	private boolean intaking;
	private boolean releasing;
	private TalonSRX motor;
	
	/**
	 * Creates a new <code>Intake</code> subsystem
	 * @param motor the intake motor
	 */
	public Intake(TalonSRX motor)
	{
		this.motor = motor;
	}

	/**
	 * Enables the intake motor to pick up a cube
	 */
	public void enable()
	{
		if(intaking)
			return;
		intaking = true;
		releasing = false;
		
		motor.set(ControlMode.PercentOutput, 1);
	}
	
	/**
	 * Enables the intake motor to release a cube
	 */
	public void release()
	{
		if(releasing)
			return;
		intaking = false;
		releasing = true;
		
		motor.set(ControlMode.PercentOutput, -1);
	}
	
	/**
	 * Disables the intake motor
	 */
	public void disable()
	{
		if(!intaking && !releasing)
			return;
		intaking = releasing = false;
		
		motor.set(ControlMode.PercentOutput, 0);
	}
}
