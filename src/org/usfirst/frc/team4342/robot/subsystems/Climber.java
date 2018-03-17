package org.usfirst.frc.team4342.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Talon;

/**
 * Climber subsystem to climb the tower
 */
public class Climber extends SubsystemBase 
{
	private boolean enabled;
	private Talon motor1, motor2;
	
	/**
	 * Creates a new <code>Climber</code> subsystem
	 * @param motor the motor to control the winch
	 */
	public Climber(Talon motor1, Talon motor2)
	{
		this.motor1 = motor1;
		this.motor2 = motor2;
	}
	
	/**
	 * Enables the winch
	 */
	public void enable()
	{
		if(enabled)
			return;
		enabled = true;
		
		motor1.set(1);
		motor2.set(1);
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
		
		motor1.set(0);
		motor2.set(0);
	}
}