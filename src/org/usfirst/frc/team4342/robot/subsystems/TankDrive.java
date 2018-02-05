package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.Constants;
import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.commands.DriveTankWithJoysticks;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Tank Drive subsystem
 */
public class TankDrive extends DriveTrainBase
{
	private Spark fr, fl, rr, rl;
	private Encoder left, right;
	
	private boolean invertRight, invertLeft;
	private double direction, leftOutput, rightOutput;
	
	/**
	 * Creates a new <code>TankDrive</code> subsystem
	 * @param frontRight the front right motor of the drive train
	 * @param frontLeft the front left motor of the drive train
	 * @param rearRight the rear right motor of the drive train
	 * @param rearLeft the right left motor of the drive train
	 * @param navx the NavX board
	 * @param left the left encoder
	 * @param right the right encoder
	 */
	public TankDrive(Spark frontRight, Spark frontLeft, Spark rearRight, Spark rearLeft, AHRS navx, Encoder left, Encoder right)
	{
		super(navx);

		this.fr = frontRight;
		this.fl = frontLeft;
		this.rr = rearRight;
		this.rl = rearLeft;
		this.left = left;
		this.right = right;

		setPID(Constants.Drive.P, Constants.Drive.I, Constants.Drive.D);
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
	 * {@inheritDoc}
	 */
	@Override
	public void initSendable(SendableBuilder builder) 
	{
		super.initSendable(builder);

		builder.setSmartDashboardType("Tank");
		builder.addDoubleProperty("Direction", () -> direction, null);
		builder.addDoubleProperty("LeftOutput", () -> leftOutput, null);
		builder.addDoubleProperty("RightOutput", () -> rightOutput, null);
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
		this.setDirection(direction);
		this.setHeading(yaw);
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
	 * Sets the left and right sides of the drive train 
	 * to the specified outputs
	 * @param left the output for the left side
	 * @param right the output for the right side
	 */
	public void set(double left, double right)
	{
		leftOutput = invertLeft ? -normalizeOutput(left) : normalizeOutput(left);
		rightOutput = invertRight ? -normalizeOutput(right) : normalizeOutput(right);
		
		fr.set(rightOutput);
		fl.set(leftOutput);
		rr.set(rightOutput);
		rl.set(leftOutput);
	}
	
	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stop()
	{
		disablePID();
		set(0, 0);
		direction = 0;
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
			Math.abs(right.getDistance()),
			Math.abs(left.getDistance())
		};
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double remainingDistance(double distance, double[] distances) {
		final double initialRight = distances[0];
		final double initialLeft = distances[1];

		final double CURRENT_RIGHT_VAL = Math.abs(right.getDistance());
		final double CURRENT_LEFT_VAL = Math.abs(left.getDistance());
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
