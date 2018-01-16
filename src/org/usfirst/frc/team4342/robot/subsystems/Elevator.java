package org.usfirst.frc.team4342.robot.subsystems;


import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.commands.ElevateWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class Elevator extends SubsystemBase
{
	
	private static final double P = 0.0, I = 0.0, D = 0.0;
	
	private PIDController elevatePID;
	private TalonSRX motor;
	private Encoder encoder;
	private DigitalInput ls;
	
	public Elevator(TalonSRX motor, Encoder encoder, DigitalInput ls) {
		this.motor = motor;
		this.encoder = encoder;
		this.ls = ls;
		
		elevatePID = new PIDController(P, I, D, encoder, new PIDOutputClass(motor));
		elevatePID.setInputRange(0, 80);
		elevatePID.setOutputRange(-1, 1);
		elevatePID.setPercentTolerance(2);
	}
	
	public void setPID(double P, double I, double D)
	{
		elevatePID.setPID(P, I, D);
	}
	
	public void set(double output)
	{
		disablePID();
		motor.set(ControlMode.PercentOutput, output);
	}
	
	public void setSetpoint(double distance)
	{
		elevatePID.enable();
		elevatePID.setSetpoint(distance);
	}
	
	public void disablePID()
	{
		if(elevatePID.isEnabled())
			elevatePID.disable();
	}
	
	public void stop()
	{
		set(0);
	}
	
	public double getDistance()
	{
		return encoder.getDistance();
	}
	
	public void reset() 
	{
		encoder.reset();
	}
	
	public boolean isAtBottom()
	{
		return ls.get();
	}
	
	public boolean isAtSetpoint()
	{
		return elevatePID.onTarget();
	}
	
	@Override
	protected void initDefaultCommand()
	{
		OI oi = OI.getInstance();
		this.setDefaultCommand(new ElevateWithJoystick(oi.ElevatorStick, oi.Elevator));
	}
	
	private class PIDOutputClass implements PIDOutput {
		private TalonSRX motor;
		public PIDOutputClass(TalonSRX motor) {
			this.motor = motor;
		}
		
		@Override
		public void pidWrite(double output) {
			motor.set(ControlMode.PercentOutput, output);
		}
	}
}
