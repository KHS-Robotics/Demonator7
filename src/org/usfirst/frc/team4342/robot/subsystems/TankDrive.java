package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.commands.DriveTankWithJoysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;

/**
 * Tank Drive subsystem
 */
public class TankDrive extends DriveTrainBase
{
	private static final double P = 0.0, I = 0.0, D = 0.0;
	
	private TalonSRX fr, fl, rr, rl;
	private Encoder left, right;
	
	private boolean invertRight, invertLeft;
	private double direction;
	
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
		super(navx);

		this.fr = fr;
		this.fl = fl;
		this.rr = rr;
		this.rl = rl;
		this.left = left;
		this.right = right;

		setPID(P, I, D);
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
		this.setDefaultCommand(new DriveTankWithJoysticks(oi.LeftDriveStick, oi.RightDriveStick, oi.Drive));
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
	 * Orients the robot to a certain yaw and then goes straight
	 * @param direction the speed to go straight ranging from -1.0 to 1.0
	 * @param yaw the yaw to orient and hold
	 */
	@Override
	public void goStraight(double direction, double yaw)
	{
		this.setHeading(normalizeYaw(yaw));
		this.setDirection(direction);
		enablePID();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void resetNavX() 
	{
		super.resetNavX();
	}
	
	/**
	 * Sets the neutral mode for the drive train
	 * @param mode the neutral mode
	 * @see NeutralMode
	 */
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
	
	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stop()
	{
		set(0, 0);
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
	 * Sets the output of the drive train to go forwards or backwards
	 * when using the PID controller to hold a specified yaw
	 * @param direction the output ranging from -1.0 to 1.0
	 */
	public void setDirection(double direction)
	{
		this.direction = direction;
	}

	/**
	 * Gets the right and left distance
	 * @return an array of two elements, with the
	 * first element being the right distance and
	 * the second element being the left distance
	 */
	@Override
	public double[] getAllDistances() {
		return new double[] {
			Math.abs(getRightDistance()),
			Math.abs(getLeftDistance())
		};
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double remainingDistance(double distance, double[] distances) {
		final double initialRight = distances[0];
		final double initialLeft = distances[1];

		final double CURRENT_RIGHT_VAL = Math.abs(getRightDistance());
		final double CURRENT_LEFT_VAL = Math.abs(getLeftDistance());
		final double DELTA_RIGHT = Math.abs(CURRENT_RIGHT_VAL - initialRight);
		final double DELTA_LEFT = Math.abs(CURRENT_LEFT_VAL - initialLeft);
		
		final double AVERAGE = (DELTA_RIGHT + DELTA_LEFT) / 2;
		
		final double REMAINING = distance - AVERAGE;
		
		return REMAINING;
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
}
