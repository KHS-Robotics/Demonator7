package org.usfirst.frc.team4342.robot.subsystems;


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
	private DigitalInput ls;
	
	public Elevator(TalonSRX motor, Encoder encoder, DigitalInput ls) {
		this.motor = motor;
		this.ls = ls;
		
		elevatePID = new PIDController(P, I, D, encoder, new PIDOutputClass(motor));
		elevatePID.setInputRange(0, 80);
		elevatePID.setOutputRange(-1, 1);
		elevatePID.setPercentTolerance(2);
	}
	
	public void set(double output)
	{
		if(elevatePID.isEnabled())
			elevatePID.disable();
		
		motor.set(ControlMode.PercentOutput, output);
	}
	
	public void setSetpoint(int distance)
	{
		elevatePID.enable();
		elevatePID.setSetpoint(distance);
	}
	
	public void stop()
	{
		if(elevatePID.isEnabled())
			elevatePID.disable();
	}
	
	public boolean isAtBottom()
	{
		return ls.get();
	}
	
	public boolean isAtSetpoint()
	{
		return elevatePID.onTarget();
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
