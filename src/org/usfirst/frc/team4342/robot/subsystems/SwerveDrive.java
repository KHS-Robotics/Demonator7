package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Swerve Drive subsystem
 */
public class SwerveDrive extends DriveTrainBase {
	private static class Constants {
		static boolean DEBUG = true;

		// Dimensions
		static final double L = 32.3, W = 23.5; // vehicle’s wheelbase and trackwidth
		static final double R = Math.sqrt((L*L) + (W*W));
		static final double L_OVER_R = L / R, W_OVER_R = W / R;

		// Pivot PID values
		static final double P = 0.0, I = 0.0, D = 0.0;
		static final double TOLERANCE = 2.0; // degrees

		// For PID and Voltage to Angle conversion
		static final double MIN_VOLTAGE = 0.0;
		static final double MAX_VOLTAGE = 5.0;
		static final double DELTA_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE;
	}
	
	private double direction;
	private boolean fieldOriented;
	
	private final SwerveModule fr;
	private final SwerveModule fl;
	private final SwerveModule rr;
	private final SwerveModule rl;
	
	/**
	 * Creates a new <code>SwerveDrive</code> subsystem
	 * @param fr the front right swerve module
	 * @param fl the front left swerve module
	 * @param rr the rear right swerve module
	 * @param rl the rear left swerve module
	 * @param navx the NavX
	 */
	public SwerveDrive(SwerveModule fr, SwerveModule fl, SwerveModule rr, SwerveModule rl, AHRS navx) {
		super(navx);

		this.fr = fr;
		this.fl = fl;
		this.rr = rr;
		this.rl = rl;

		setPID(Constants.P, Constants.I, Constants.D);
	}
	
	/**
	 * Sets the default command to <code>DriveSwerveWithJoystick</code>
	 */
	@Override
	protected void initDefaultCommand() {
//		OI oi = OI.getInstance();
//		this.setDefaultCommand(new DriveSwerveWithJoystick(oi.DriveStick, oi.Drive));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void pidWrite(double output) {
		if(Constants.DEBUG)
			Logger.debug("Swerve Drive pidWrite output=" + output);

		this.set(0, direction, output);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void goStraight(double direction, double yaw) {
		if(Constants.DEBUG)
			Logger.debug("SwerveDrive goStraight direction=" + direction + " yaw=" + yaw);

		this.direction = direction;
		this.setHeading(yaw);
	}
	
	/**
	 * Sets field oriented vs. robot oriented
	 * @param flag true for field oriented, false for robot oriented
	 */
	public void setFieldOriented(boolean flag) {
		if(Constants.DEBUG)
			Logger.debug("SwerveDrive setFieldOriented flag=" + flag);

		fieldOriented = flag;
	}
	
	/**
	 * Sets the x, y and z for the drive train. For information
	 * on the math, see Either's Derivation of swerve formulas
	 * using Inverse Kinematics: http://www.chiefdelphi.com/media/papers/download/3028
	 * @param x the x input (e.g., strafe)
	 * @param y the y input (e.g., forward/backward)
	 * @param z the z input (e.g., twist)
	 */
	public void set(double x, double y, double z) {
		double fwd = -y;
		double str = x;
		final double rcw = z;
		
		if(fieldOriented) {
			final double currentAngle = this.getAngle();
			final double TEMP = fwd*Math.cos(currentAngle) + str*Math.sin(currentAngle);
			str = -fwd*Math.sin(currentAngle) + str*Math.cos(currentAngle);
			fwd = TEMP;
		}
		
		
		final double xNeg = str - (rcw*Constants.L_OVER_R);
		final double xPos = str + (rcw*Constants.L_OVER_R);
		final double yNeg = fwd - (rcw*Constants.W_OVER_R);
		final double yPos = fwd + (rcw*Constants.W_OVER_R);
		
		double frSpeed = calcMagnitude(xPos, yNeg);
		double frPivot = calcAngle(xPos, yNeg);
		
		double flSpeed = calcMagnitude(xPos, yPos);
		double flPivot = calcAngle(xPos, yPos);
		
		double rlSpeed = calcMagnitude(xNeg, yPos);
		double rlPivot = calcAngle(xNeg, yPos);
		
		double rrSpeed = calcMagnitude(xNeg, yNeg);
		double rrPivot = calcAngle(xNeg, yNeg);

		// Make sure we don't use the arctan value
		// with x=0
		if(xNeg == 0 && Math.abs(fwd) < 0.001) {
			frPivot = fr.getAngle();
			flPivot = fl.getAngle();
			rrPivot = rr.getAngle();
			rlPivot = rl.getAngle();
		}
		else if(xNeg == 0) {
			frPivot = flPivot = rrPivot = rlPivot = 180;
		}
		
		double max = frSpeed;
		if(flSpeed > max)
			max = flSpeed;
		if(rlSpeed > max)
			max = rlSpeed;
		if(rrSpeed > max)
			max = rrSpeed;
		
		if(max > 1) {
			frSpeed /= max;
			flSpeed /= max;
			rlSpeed /= max;
			rrSpeed /= max;
		}

		fr.setPivot(frPivot);
		fl.setPivot(flPivot);
		rl.setPivot(rlPivot);
		rr.setPivot(rrPivot);
		
		fr.setDrive(frSpeed);
		fl.setDrive(flSpeed);
		rl.setDrive(rlSpeed);
		rr.setDrive(rrSpeed);

		if(Constants.DEBUG) {
			Logger.debug("SwerveDrive set x=" + x + " y=" + y + " z=" + z);
			Logger.debug("FL speed=" + flSpeed + " pivot=" + flPivot + " :: FR speed=" + frSpeed + " pivot=" + frPivot);
			Logger.debug("RL speed=" + rlSpeed + " pivot=" + rlPivot + " :: RL speed=" + rlSpeed + " pivot=" + rlPivot);
		}
	}
	
	/**
	 * Sets all modues to the specified output and pivot angle
	 * @param output the speed ranging from 0 to 1
	 * @param angle the angle ranging from 0 to 360
	 */
	public void setAll(double output, double angle) {
		if(Constants.DEBUG)
			Logger.debug("SwerveDrive setAll output=" + output + " angle=" + angle);

		fr.setPivot(angle);
		fl.setPivot(angle);
		rr.setPivot(angle);
		rl.setPivot(angle);
		
		fr.setDrive(output);
		fl.setDrive(output);
		rr.setDrive(output);
		rl.setDrive(output);
	}

	/**
	 * Sets the drive output for all modules
	 * @param output the speed ranging from -1 to 1
	 */
	public void setDrive(double output) {
		if(Constants.DEBUG)
			Logger.debug("SwerveDrive serDrive output=" + output);

		fr.setDrive(output);
		fl.setDrive(output);
		rr.setDrive(output);
		rl.setDrive(output);
	}

	/**
	 * Sets the pivot angle for all modules
	 * @param angle the pivot angle
	 */
	public void setPivot(double angle) {
		if(Constants.DEBUG)
			Logger.debug("SwerveDrive setPivot angle=" + angle);
			
		fr.setPivot(angle);
		fl.setPivot(angle);
		rr.setPivot(angle);
		rl.setPivot(angle);
	}

	/**
	 * Gets if all pivot motors are at their setpoint
	 * @return true if all pivot motors are at their setpoint, false otherwise
	 */
	public boolean pivotAtSetpoint() {
		return fr.pivotAtSetpoint() && fl.pivotAtSetpoint() && rr.pivotAtSetpoint() && fl.pivotAtSetpoint();
	}
	
	/**
	 * Stops drive motor on all modules
	 */
	public void stop() {
		fr.stop();
		fl.stop();
		rr.stop();
		rl.stop();

		direction = 0;
	}

	/**
	 * Gets the current front right drive distance
	 * @return the current front right drive distance
	 */
	public double getFRDistance() {
		return fr.getDistance();
	}

	/**
	 * Gets the current front left drive distance
	 * @return the current front left drive distance
	 */
	public double getFLDistance() {
		return fl.getDistance();
	}

	/**
	 * Gets the current rear right drive distance
	 * @return the current rear right drive distance
	 */
	public double getRRDistance() {
		return rr.getDistance();
	}

	/**
	 * Gets the current rear left drive distance
	 * @return the current rear left drive distance
	 */
	public double getRLDistance() {
		return rl.getDistance();
	}

	/**
	 * Gets all drive distances
	 * @return an array of four elements, in the
	 * following order: fr, fl, rr, rl
	 */
	@Override
	public double[] getAllDistances() {
		return new double[] {
			Math.abs(fr.getDistance()),
			Math.abs(fl.getDistance()),
			Math.abs(rr.getDistance()),
			Math.abs(rl.getDistance())
		};
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double remainingDistance(double distance, double[] distances) {
		final double initalFR = distances[0];
		final double initalFL = distances[1];
		final double initalRR = distances[2];
		final double initalRL = distances[3];

		final double frDist = Math.abs(fr.getDistance());
		final double flDist = Math.abs(fl.getDistance());
		final double rrDist = Math.abs(rr.getDistance());
		final double rlDist = Math.abs(rl.getDistance());

		final double dFR = Math.abs(frDist - initalFR);
		final double dFL = Math.abs(flDist - initalFL);
		final double dRR = Math.abs(rrDist - initalRR);
		final double dRL = Math.abs(rlDist - initalRL);

		final double AVERAGE = (dFR + dFL + dRR + dRL) / 4;
		final double REMAINING = distance - AVERAGE;

		return REMAINING;
	}

	/**
	 * Cartesian to Polar calculation for the magnitude, r
	 * @param x the x coordinate
	 * @param y the y coordinate
	 * @return the magnitude of polar coordinate
	 */
	private static double calcMagnitude(double x, double y) {
		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	/**
	 * Cartesian to Polar calculation for the angle, theta
	 * @param x the x coordinate
	 * @param y the y coordinate
	 * @return the angle of the polar coordinate
	 */
	private static double calcAngle(double x, double y) {
		return Math.toDegrees(Math.atan2(y, x)) + 180; // add 180 b/c straight ahead is 2.5v = 180 deg
	}
	
	/**
	 * Swerve Module
	 */
	public static class SwerveModule {
		private PIDController pivotPID;
		private double offset;
		private boolean flipDrive;

		private TalonSRX drive, pivot;
		private Encoder driveEnc;
		private AnalogInput pivotEnc;
		
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
		
		/**
		 * Swerve Module
		 * @param drive the drive motor for translation
		 * @param driveEnc the encoder for the drive motor
		 * @param pivot the pivot motor for rotation
		 * @param pivotEnc the analog input for the pivot motor
		 */
		public SwerveModule(TalonSRX drive, Encoder driveEnc, TalonSRX pivot, AnalogInput pivotEnc) {
			this.drive = drive;
			this.driveEnc = driveEnc;
			this.pivot = pivot;
			this.pivotEnc = pivotEnc;
			
			pivotPID = new PIDController(
				Constants.P, 
				Constants.I, 
				Constants.D, 
				pivotEnc, 
				new PIDOutputClass(pivot)
			);
			
			pivotPID.setInputRange(Constants.MIN_VOLTAGE, Constants.MAX_VOLTAGE);
			pivotPID.setOutputRange(-1, 1);
			pivotPID.setAbsoluteTolerance(toVoltage(Constants.TOLERANCE));
			pivotPID.setContinuous();
		}

		/**
		 * Sets the offset for the pivot
		 * @param offset the offset for the pivot
		 */
		public void setOffset(double offset) {
			this.offset = offset;
		}
		
		/**
		 * Sets the speed for the drive motor
		 * @param output the speed ranging from 0 to 1
		 */
		public void setDrive(double output) {
			output = flipDrive ? -output : output;
			drive.set(ControlMode.PercentOutput, output);
		}
		
		/**
		 * Sets the pivot angle
		 * @param angle the pivot angle ranging from 0 to 360
		 */
		public void setPivot(double angle) {
			angle %= 360;

			// Check if complementary angle is closer
			double dAngle = angle - getAngle();
			flipDrive = dAngle >= 90 && dAngle <= 270;

			// if it is closer then use
			// complementary (e.g., add 180)
			if(flipDrive)
				angle += 180;

			if(Constants.DEBUG)
				Logger.debug("SwerveModule setPivot flipDrive=" + flipDrive + " angle=" + angle);

			pivotPID.setSetpoint(toVoltage(angle));
			pivotPID.enable();
		}

		/**
		 * Gets the pivot angle in degrees
		 * @return the pivot angle in degrees
		 */
		public double getAngle() {
			double angle = toAngle(pivotEnc.getAverageVoltage());
			double normalized = angle % 360;
			return normalized;
		}

		/**
		 * Gets the drive distance
		 * @return the drive distance
		 */
		public double getDistance() {
			return driveEnc.getDistance();
		}

		/**
		 * Gets if the pivot motor is at its setpoint
		 * @return true if the pivot motor is at its setpoint, false otherwise
		 */
		public boolean pivotAtSetpoint() {
			return pivotPID.onTarget();
		}
		
		/**
		 * Stops the swerve module
		 */
		public void stop() {
			drive.set(ControlMode.PercentOutput, 0);
			if(pivotPID.isEnabled())
				pivotPID.disable();
		}

		/**
		 * Coverts a voltage from the analog input to an angle
		 * @param voltage the voltage
		 * @param offset the offset
		 * @return the angle of the analog input (e.g., pivot angle)
		 */
		private double toAngle(double voltage) {
			return ((360.0 * (voltage - Constants.MIN_VOLTAGE) / Constants.DELTA_VOLTAGE) + 360.0 - offset);
		}

		/**
		 * Converts an angle to voltage for the analog input
		 * @param angle the angle in degrees
		 * @return the voltage of the analog input based on the angle
		 */
		private double toVoltage(double angle) {
			angle %= 360;
			angle += 360; // add 360 to ensure formula gives a value from 0 to 5
			return (Constants.MIN_VOLTAGE + (Constants.DELTA_VOLTAGE*(offset + angle - 360))/360.0);
		}
	}
}
