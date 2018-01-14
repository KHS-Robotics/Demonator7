package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.commands.DriveTankWithJoysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TankDrive extends SubsystemBase implements PIDSource, PIDOutput
{
	private static final double P = 0.0, I = 0.0, D = 0.0;
	
	private TalonSRX fr, fl, rr, rl;
	private AHRS navx;
	private Encoder left, right;
	private PIDController yawPID;
	
	private boolean invertRight, invertLeft;
	private double direction;
	private double offset;
	private PIDSourceType pidSourceType;
	
	/**
	 * Creates a new <code>TankDrive</code> subsystem
	 * @param fr the front right motor of the drive train
	 * @param fl the front left motor of the drive train
	 * @param rr the rear right motor of the drive train
	 * @param rl the right left motor of the drive train
	 * @param navx the NavX board
	 * @param left the left encoder
	 * @param right the right encoder
	 */
	public TankDrive(TalonSRX fr, TalonSRX fl, TalonSRX rr, TalonSRX rl, AHRS navx, Encoder left, Encoder right)
	{
		this.fr = fr;
		this.fl = fl;
		this.rr = rr;
		this.rl = rl;
		this.navx = navx;
		this.left = left;
		this.right = right;
		
		setPIDSourceType(PIDSourceType.kDisplacement);
		
		yawPID = new PIDController(P, I, D, this, this);
		yawPID.setInputRange(-180.0, 180.0);
		yawPID.setOutputRange(-1.0, 1.0);
		yawPID.setContinuous();
		yawPID.setAbsoluteTolerance(2);
		disablePID();
	}
	
	public void setRightInverted(boolean inverted)
	{
		this.invertRight = inverted;
	}
	
	public void setLeftInverted(boolean inverted)
	{
		this.invertLeft = inverted;
	}
	
	public void setNeutralMode(NeutralMode mode)
	{
		fr.setNeutralMode(mode);
		fl.setNeutralMode(mode);
		rr.setNeutralMode(mode);
		rl.setNeutralMode(mode);
	}
	
	/**
	 * Sets the left and right sides of the drive train 
	 * to the specified outputs
	 * @param left the output for the left side
	 * @param right the output for the right side
	 */
	public void set(double left, double right)
	{
		disablePID();
		
		left = invertLeft ? -normalizeOutput(left) : normalizeOutput(left);
		right = invertRight ? -normalizeOutput(right) : normalizeOutput(right);
		
		fr.set(ControlMode.PercentOutput, right);
		fl.set(ControlMode.PercentOutput, left);
		rr.set(ControlMode.PercentOutput, right);
		rl.set(ControlMode.PercentOutput, left);
	}
	
	public void stop()
	{
		this.disablePID();
		this.set(0, 0);
	}

	/**
	 * Gets the right drive encoder's distance
	 * @return the right drive encoder's distance
	 */
	public double getRightDistance()
	{
		return right.getDistance();
	}
	
	/**
	 * Gets the left drive encoder's distance
	 * @return the left drive encoder's distance
	 */
	public double getLeftDistance()
	{
		return left.getDistance();
	}
	
	/**
	 * Resets the encoders, should only be used when testing
	 */
	public void resetEncoders()
	{
		left.reset();
		right.reset();
	}
	
	/**
	 * Resets the NavX, should only be used when testing
	 */
	public void resetNavX()
	{
		navx.reset();
	}
	
	/**
	 * Gets the current heading of the robot
	 * @return the current heading of the robot ranging from -180.0 to 180.0 degrees
	 */
	public double getHeading()
	{
		return normalizeYaw(navx.getYaw() + offset);
	}
	
	/**
	 * Sets the internal PID conroller's setpoint to the specified yaw and enables PID
	 * @param yaw the yaw to orient the robot to
	 */
	public void setHeading(double yaw)
	{
		yawPID.setSetpoint(normalizeYaw(yaw));
		enablePID();
	}
	
	/**
	 * Enables the internal PID controller
	 */
	public void enablePID()
	{
		yawPID.enable();
	}
	
	/**
	 * Disables the internal PID controller if it's enabled
	 */
	public void disablePID()
	{
		if(yawPID.isEnabled())
		{
			yawPID.disable();
			direction = 0;
		}
	}
	
	/**
	 * Returns if the internal PID controller is enabled
	 * @return true if the internal PID controller is enabled, false otherwise
	 */
	public boolean pidEnabled()
	{
		return yawPID.isEnabled();
	}
	
	/**
	 * Orients the robot to a certain yaw and then goes straight
	 * @param direction the speed to go straight ranging from -1.0 to 1.0
	 * @param yaw the yaw to orient and hold
	 */
	public void goStraight(double direction, double yaw)
	{
		this.setHeading(normalizeYaw(yaw));
		this.setDirection(direction);
		enablePID();
	}

	/**
	 * Sets the output of the drive train to go forwards or backwards
	 * when using the PID controller to hold a specified yaw
	 * @param direction the output ranging from -1.0 to 1.0
	 */
	public void setDirection(double direction)
	{
		this.direction = direction;
	}
	
	/**
	 * Sets the yaw offset to return the proper values from the NavX
	 * @param offset the offset ranging from -180.0 to 180.0
	 */
	public void setYawOffset(double offset)
	{
		this.offset = offset;
	}
	
	/**
	 * Gets if the internal PID controller is on target with its setpoint
	 * within a tolerance of three degrees
	 * @return true if the internal PID controller is at its setpoint, false otherwise
	 */
	public boolean onTarget()
	{
		return yawPID.onTarget();
	}
	
	/**
	 * Sets the internal PID controller's PIDSourceType
	 */
	@Override
	public void setPIDSourceType(PIDSourceType pidSource)
	{
		pidSourceType = pidSource;
	}
	
	/**
	 * Gets the internal PID Controller's current PIDSourceType
	 * @return the internal PID Controller's current PIDSourceType
	 */
	@Override
	public PIDSourceType getPIDSourceType()
	{
		return pidSourceType;
	}
	
	/**
	 * Gets the heading of the robot
	 * @return the current yaw of the robot
	 */
	@Override
	public double pidGet()
	{
		if(pidSourceType == PIDSourceType.kRate)
			return navx.getRate();
		else
			return this.getHeading();
	}
	
	/**
	 * Overridden method to specify how the PIDController should output
	 * to the drive train
	 */
	@Override
	public void pidWrite(double output)
	{
		double left = direction + output;
		double right = direction - output;
		
		this.set(left, right);
	}
	
	/**
	 * Calculates the remaining distance the robot needs to drive before
	 * reaching the desired distance
	 * @param distance the desired distance (in inches)
	 * @param initialLeft the initial left encoder distance (in inches, basically a snapshot of {@link #getLeftDistance()})
	 * @param initialRight the initial right encoder distance (in inches, basically a snapshot of {@link #getRightDistance()})
	 * @return the remaining distance the robot needs to drive
	 */
	public double remainingDistance(double distance, double initialLeft, double initialRight)
	{
		final double CURRENT_RIGHT_VAL = Math.abs(getRightDistance());
		final double CURRENT_LEFT_VAL = Math.abs(getLeftDistance());
		final double DELTA_RIGHT = Math.abs(CURRENT_RIGHT_VAL - initialRight);
		final double DELTA_LEFT = Math.abs(CURRENT_LEFT_VAL - initialLeft);
		
		final double AVERAGE = (DELTA_RIGHT + DELTA_LEFT) / 2;
		
		final double REMAINING = distance - AVERAGE;
		
		return REMAINING;
	}
	
	/**
	 * <p>Sets the default command to <code>DriveTankWithJoysticks</code></p>
	 * 
	 * {@inheritDoc}
	 * @see DriveTankWithJoysticks
	 */
	@Override
	protected void initDefaultCommand()
	{
		OI oi = OI.getInstance();
		this.setDefaultCommand(new DriveTankWithJoysticks(oi.LeftDriveStick, oi.RightDriveStick, oi.TankDrive));
	}
	
	/**
	 * Internal function to normalize a motor output
	 * @param output the unnormalized output
	 * @return the normalized output ranging from -1.0 to 1.0
	 */
	private static double normalizeOutput(double output)
	{
		if(output > 1)
			return 1.0;
		else if(output < -1)
			return -1.0;
		return output;
	}
	
	/**
	 * Internal function to normalize yaw
	 * @param yaw the unnormalized yaw
	 * @return the normalized yaw ranging from -180.0 o 180.0
	 */
	private static double normalizeYaw(double yaw)
	{
		while(yaw >= 180)
			yaw -= 360;
		while(yaw <= -180)
			yaw += 360;
		
		return yaw;
	}
}
