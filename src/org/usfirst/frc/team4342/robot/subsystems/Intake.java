package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.RobotMap;
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
	
	private double limiter;
	
	/**
	 * Creates a new <code>Intake</code> subsystem
	 * @param motor the intake motor
	 */
	public Intake(Talon motorLeft, Talon motorRight)
	{
		this.motorLeft = motorLeft;
		this.motorRight = motorRight;
	}
	

	@Override 
	protected void initDefaultCommand() {
		final OI oi = OI.getInstance();
		this.setDefaultCommand(new IntakeWithJoystick(oi.Intake,oi.SwitchBox));
	}
	
	public void set(double output, double x) {
		if(intaking || releasing)
			return;
		
		if(OI.getInstance().PDP.getCurrent(RobotMap.INTAKE_MOTOR_LEFT)> 8)
		{
			limiter = .8;
		}
		else
		{
			limiter = 1;
		}
		
		if(x < 0)
		{
			motorLeft.set((output + Math.abs(x))*limiter);
			motorRight.set(output * limiter);
		}
		else
		{
			motorLeft.set(output * limiter);
			motorRight.set((output + Math.abs(x))*limiter);
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
		
		if(OI.getInstance().PDP.getCurrent(RobotMap.INTAKE_MOTOR_LEFT)>10)
		{
			motorRight.set(.65);
			motorLeft.set(.65);
		}
		else
		{
			motorLeft.set(.75);
			motorRight.set(.75);
		}
		
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
		
		if((Math.abs(OI.getInstance().PDP.getCurrent(RobotMap.INTAKE_MOTOR_LEFT) - OI.getInstance().PDP.getCurrent(RobotMap.INTAKE_MOTOR_RIGHT))/20) <= 0.1)
		{
			if(OI.getInstance().PDP.getCurrent(RobotMap.INTAKE_MOTOR_LEFT) > OI.getInstance().PDP.getCurrent(RobotMap.INTAKE_MOTOR_RIGHT))
			{
				motorRight.set(.85);
				motorLeft.set(.75);
			}
			else
			{
				motorRight.set(.75);
				motorLeft.set(.85);
			}
		}
		else
		{
			motorLeft.set(.75);
			motorRight.set(.75);
		}
		
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
