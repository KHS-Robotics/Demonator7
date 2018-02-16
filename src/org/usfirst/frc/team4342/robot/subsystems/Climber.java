package org.usfirst.frc.team4342.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;

/**
 * Climber subsystem to climb the tower
 */
public class Climber extends SubsystemBase 
{
	private boolean enabled;
	private Spark motor;
	
	/**
	 * Creates a new <code>Climber</code> subsystem
	 * @param motor the motor to control the winch
	 */
	public Climber(Spark motor)
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
		
		motor.set(1);
	}
	
	/**
	 * Disables the winch
	 */
	@Override
	public void stop()
	{
		if(!enabled)
			return;
		enabled = false;
		
		motor.set(0);
	}
}