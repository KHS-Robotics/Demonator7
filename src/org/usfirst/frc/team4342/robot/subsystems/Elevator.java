package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.Constants;
import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.commands.ElevateWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Elevator subsystem
 */
public class Elevator extends PIDSubsystem
{
	private double p, i, d;
	
	private TalonSRX motor;
	private Encoder encoder;
	private DigitalInput ls;
	
	/**
	 * Creates a new <code>Elevator</code> subsystem
	 * @param motor the motor to move the elevator
	 * @param encoder the encoder to keep track of the elevator's height
	 * @param ls the limit switch at the bottom of the elevator
	 */
	public Elevator(TalonSRX motor, Encoder encoder, DigitalInput ls) 
	{
		super(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D);
		setInputRange(0, 80);
		setOutputRange(-1, 1);
		setAbsoluteTolerance(0.5);

		this.motor = motor;
		this.encoder = encoder;
		this.ls = ls;
	}

	/**
	 * Gets the encoder distance
	 * @return the encoder distance
	 */
	@Override
	protected double returnPIDInput() 
	{
		return encoder.getDistance();
	}

	/**
	 * Sets the calculated output to the motor
	 */
	@Override
	protected void usePIDOutput(double output) 
	{
		set(output);
	}

	/**
	 * Sets the default command to <code>ElevateWithJoystick</code>
	 */
	@Override
	protected void initDefaultCommand()
	{
		OI oi = OI.getInstance();
		this.setDefaultCommand(new ElevateWithJoystick(oi.SwitchBox, oi.Elevator));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void initSendable(SendableBuilder builder) 
	{
		super.initSendable(builder);

		builder.setSmartDashboardType("Elevator");
		builder.setSafeState(this::stop);
		builder.addDoubleProperty("Height", this::getPosition, null);
		builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
		builder.addBooleanProperty("OnTarget", this::onTarget, null);
		builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("D", this::getD, this::setD);
		builder.addBooleanProperty("IsAtBottom", this::isAtBottom, null);
	}

	/**
	 * Stops the elevator
	 */
	public void stop()
	{
		disable();
		set(0);
	}

	/**
	 * Sets the PID values for the internal PID controller for elevator height.
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivatve value
	 */
	public void setPID(double p, double i, double d)
	{
		this.p = p;
		this.i = i;
		this.d = d;
		this.getPIDController().setPID(p, i, d);
	}

	/**
     * Sets the P value for the internal PID controller for height.
     * @param p the proportional value
     */
    public void setP(double p) {
        this.p = p;
        setPID(p, i, d);
    }

    /**
     * Gets the proportional value for the internal PID controller for height.
     * @return the proportional value
     */
    public double getP() {
        return p;
    }

    /**
     * Sets the I value for the internal PID controller for height.
     * @param i the integral value
     */
    public void setI(double i) {
        this.i = i;
        setPID(p, i, d);
    }

    /**
     * Gets the integral value for the internal PID controller for height.
     * @return the integral value
     */
    public double getI() {
        return i;
    }

    /**
     * Sets the D value for the intenral PID controller for height.
     * @param d the derivative value
     */
    public void setD(double d) {
        this.d = d;
        setPID(p, i, d);
    }

    /**
     * Gets the derivative value for the internal PID controller for height.
     * @return the derivative value
     */
    public double getD() {
        return d;
	}
	
	/**
	 * Sets the neutral mode for the motor
	 * @see com.ctre.phoenix.motorcontrol.NeutralMode
	 */
	public void setNeutralMode(NeutralMode mode)
	{
		motor.setNeutralMode(mode);
	}
	
	/**
	 * Sets the output of the elevator motor
	 * @param output the desired output ranging from -1 to 1 (negative
	 * for down, positive for up)
	 */
	public void set(double output)
	{
		motor.set(ControlMode.PercentOutput, output);
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
}
