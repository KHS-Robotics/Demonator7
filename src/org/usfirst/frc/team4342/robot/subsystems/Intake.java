package org.usfirst.frc.team4342.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;

/**
 * Intake subsystem to intake and release cubes
 */
public class Intake extends SubsystemBase 
{
	private boolean intaking;
	private boolean releasing;
	private Spark motor;
	
	/**
	 * Creates a new <code>Intake</code> subsystem
	 * @param motor the intake motor
	 */
	public Intake(Spark motor)
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
		
		motor.set(0.50);
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
		
		motor.set(-0.50);
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
