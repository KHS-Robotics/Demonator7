package org.usfirst.frc.team4342.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Climber subsystem to climb the tower
 */
public class Climber extends SubsystemBase 
{
	private boolean enabled;
	private TalonSRX motor1, motor2;
	
	/**
	 * Creates a new <code>Climber</code> subsystem
	 * @param motor the motor to control the winch
	 */
	public Climber(TalonSRX motor1, TalonSRX motor2)
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
		
		motor1.set(ControlMode.PercentOutput, 1);
		motor2.set(ControlMode.PercentOutput, 1);
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
		
		motor1.set(ControlMode.PercentOutput, 0);
		motor2.set(ControlMode.PercentOutput, 0);
	}
}