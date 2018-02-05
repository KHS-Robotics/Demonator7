package org.usfirst.frc.team4342.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

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
	 * {@inheritDoc}
	 */
	@Override
	public void initSendable(SendableBuilder builder) 
	{
		super.initSendable(builder);

		builder.setSmartDashboardType("Intake");
		builder.setSafeState(this::disable);
		builder.addBooleanProperty("Intaking", () -> intaking, null);
		builder.addBooleanProperty("Releasing", () -> releasing, null);
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
		
		motor.set(1);
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
		
		motor.set(-1);
	}
	
	/**
	 * Disables the intake motor
	 */
	public void disable()
	{
		if(!intaking && !releasing)
			return;
		intaking = releasing = false;
		
		motor.set(0);
	}
}
