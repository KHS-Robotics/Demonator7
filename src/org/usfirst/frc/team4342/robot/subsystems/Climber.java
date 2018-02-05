package org.usfirst.frc.team4342.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

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
	 * {@inheritDoc}
	 */
	@Override
	public void initSendable(SendableBuilder builder) 
	{
		super.initSendable(builder);

		builder.setSmartDashboardType("Climber");
		builder.setSafeState(this::disable);
		builder.addBooleanProperty("Enabled", () -> enabled, null);
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
	public void disable()
	{
		if(!enabled)
			return;
		enabled = false;
		
		motor.set(0);
	}
}