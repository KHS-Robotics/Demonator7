package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.commands.intake.IntakeWithJoystick;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * Intake subsystem to intake and release cubes
 */
public class Intake extends SubsystemBase 
{
	private boolean intaking;
	private boolean releasing;
	private boolean slow;
	private Talon motorLeft, motorRight;
	private Ultrasonic ultra;
	
	/**
	 * Creates a new <code>Intake</code> subsystem
	 * @param motor the intake motor
	 */
	public Intake(Talon motorLeft, Talon motorRight, Ultrasonic ultra)
	{
		this.motorLeft = motorLeft;
		this.motorRight = motorRight;
		this.ultra = ultra;
	}
	
	public double getUltraRange() {
		return ultra.getRangeInches();
	}

	@Override 
	protected void initDefaultCommand() {
		final OI oi = OI.getInstance();
		this.setDefaultCommand(new IntakeWithJoystick(oi.Intake,oi.SwitchBox));
	}
	
	public void set(double output, double x) {
		if(intaking || releasing)
			return;
		
		if(x < 0)
		{
			motorLeft.set(output + Math.abs(x));
			motorRight.set(output);
		}
		else
		{
			motorLeft.set(output);
			motorRight.set(output + Math.abs(x));
		}
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
		
		motorLeft.set(.75);
		motorRight.set(.75);
		
	}
	
	/**
	 * Enables the intake motor to pick up a cube (w/ profiling)
	 */
	public void profilingEnable()
	{
		if(intaking)
			return;
		intaking = true;
		releasing = false;
		
		motorLeft.set(.75);
		motorRight.set(.75);
		
//		if(motorLeft)
//		{
//			
//		}
//		else if(motorRight)
//		{
//			
//		}
		
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
		
		if(slow)
		{
			motorLeft.set(-0.70);
			motorRight.set(-0.70);
		}
		else
		{
			motorLeft.set(-0.40);
			motorRight.set(-0.40);
		}
	}
	
	/**
	 * sets motors to slower setting
	 */
	public void slow()
	{
		slow = true;
	}
	
	/**
	 * returns motors to faster setting
	 */
	public void fast()
	{
		slow = false;
		
	}
	
	/**
	 * tells if the cube is intook already
	 */
	public boolean hasCube()
	{
		return (ultra.getRangeInches() <= 4);
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
		
		motorLeft.set(0);
		motorRight.set(0);
	}
}
