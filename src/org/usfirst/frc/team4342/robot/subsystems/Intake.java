package org.usfirst.frc.team4342.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;

/**
 * Intake subsystem to intake and release cubes
 */
public class Intake extends SubsystemBase 
{
	private boolean intaking;
	private boolean releasing;
	private Talon motor;
	
	/**
	 * Creates a new <code>Intake</code> subsystem
	 * @param motor the intake motor
	 */
	public Intake(Talon motor)
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
		
		motor.set(.75);
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
		
		motor.set(-0.70);
	}
	
	/**
	 * Disables the intake motor
	 */
	@Override
	public void stop()
	{
		if(!intaking && !releasing)
			return;
		intaking = releasing = false;
		
		motor.set(0);
	}
}
