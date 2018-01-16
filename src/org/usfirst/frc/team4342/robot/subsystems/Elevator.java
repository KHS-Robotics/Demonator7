package org.usfirst.frc.team4342.robot.subsystems;


import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.commands.ElevateWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * Elevator subsystem
 */
public class Elevator extends SubsystemBase
{
	private static final double P = 0.0, I = 0.0, D = 0.0;
	
	private PIDController elevatePID;
	private TalonSRX motor;
	private Encoder encoder;
	private DigitalInput ls;
	private Ultrasonic ultra;
	
	/**
	 * Creates a new <code>Elevator</code> subsystem
	 * @param motor the motor to move the elevator
	 * @param encoder the encoder to keep track of the elevator's height
	 * @param ls the limit switch at the bottom of the elevator
	 * @param ultra the ultrasonic sensor to detect cubes within our intake
	 */
	public Elevator(TalonSRX motor, Encoder encoder, DigitalInput ls, Ultrasonic ultra) {
		this.motor = motor;
		this.encoder = encoder;
		this.ls = ls;
		this.ultra = ultra;
		
		elevatePID = new PIDController(P, I, D, encoder, new PIDOutputClass(motor));
		elevatePID.setInputRange(0, 80);
		elevatePID.setOutputRange(-1, 1);
		elevatePID.setPercentTolerance(2);
	}
	
<<<<<<< HEAD
	public void setPID(double P, double I, double D)
	{
		elevatePID.setPID(P, I, D);
	}
	
=======
	/**
	 * Gets if there is a cube in the intake
	 * @return true if cube in intake, false otherwise
	 */
	public boolean hasCube()
	{
		return ultra.getRangeInches() < 6;
	}
	
	/**
	 * Sets the output of the elevator motor
	 * @param output the desired output ranging from -1 to 1 (negative
	 * for down, positive for up)
	 */
>>>>>>> branch 'master' of https://github.com/KHS-Robotics/Demonator7.git
	public void set(double output)
	{
		disablePID();
		motor.set(ControlMode.PercentOutput, output);
	}
	
	/**
	 * Sets the elevator height
	 * @param height the desired height of the elevator
	 */
	public void setSetpoint(double height)
	{
		elevatePID.enable();
		elevatePID.setSetpoint(height);
	}
	
	/**
	 * Stops the elevator
	 */
	public void stop()
	{
		set(0);
	}
	
	/**
	 * Gets the elevator's height
	 * @return the elevator's height
	 */
	public double getDistance()
	{
		return encoder.getDistance();
	}
	
	/**
	 * Resets the elevator's encoder
	 */
	public void reset() 
	{
		encoder.reset();
	}
	
	/**
	 * Gets if the elevator is at the bottom
	 * @return true if the elevator is at the bottom, false otherwise
	 */
	public boolean isAtBottom()
	{
		return ls.get();
	}
	
	/**
	 * Gets if the elevator is at its desired height
	 * @return true if the elevator is at its desired height, false otherwise
	 */
	public boolean isAtSetpoint()
	{
		return elevatePID.onTarget();
	}
	
	/**
	 * Sets the default command to <code>ElevateWithJoystick</code>
	 */
	@Override
	protected void initDefaultCommand()
	{
		OI oi = OI.getInstance();
		this.setDefaultCommand(new ElevateWithJoystick(oi.ElevatorStick, oi.Elevator));
	}

	/**
	 * Disables the internal PID Controller for the elevator
	 */
	private void disablePID()
	{
		if(elevatePID.isEnabled())
			elevatePID.disable();
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
