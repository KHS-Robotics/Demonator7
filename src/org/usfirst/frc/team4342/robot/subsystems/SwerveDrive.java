package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.Constants;
import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveSwerveWithXbox;
import org.usfirst.frc.team4342.robot.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Swerve Drive subsystem
 */
public class SwerveDrive extends DriveTrainBase {
	private static boolean DEBUG = false;

	// Dimensions in inches (vehicle's wheelbase and trackwidth)
	// measurements are from one center of pivot to another center of pivot
	private static final double L = 20.25;
	private static final double W = 25.50;
	private static final double R = Math.sqrt((L*L) + (W*W));
	private static final double L_OVER_R = L / R, W_OVER_R = W / R;

	// Pivot PID
	private static final double TOLERANCE = 2.0; // degrees

	// For PID and Voltage to Angle conversion
	private static final double MIN_VOLTAGE = 0.20;
	private static final double MAX_VOLTAGE = 4.76;
	private static final double DELTA_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE;
	
	// Saves battery
	private static final double DRIVE_OUTPUT_LIMITER = 0.85;
	
	private double xDirection, yDirection;
	private boolean fieldOriented;
	
	public final SwerveModule fr;
	public final SwerveModule fl;
	public final SwerveModule rr;
	public final SwerveModule rl;
	
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
		
		setPID(Constants.Drive.P, Constants.Drive.I, Constants.Drive.D);
	}

	/**
	 * Sets the default command to <code>DriveSwerveWithXbox</code>
	 */
	@Override
	protected void initDefaultCommand() {
		final OI oi = OI.getInstance();
		this.setDefaultCommand(new DriveSwerveWithXbox(oi.DriveController, oi.Drive));
	}

	/**
	 * Sets flag for debug statements
	 * @param flag true for debug statements to stdout, false otherwise
	 */
	public void setDebug(boolean flag) {
		DEBUG = flag;
	}

	/**
	 * Sets the neutral mode on drive train
	 * @see com.ctre.phoenix.motorcontrol.NeutralMode
	 */
	public void setNeutralMode(NeutralMode mode) {
		fr.setNeutralMode(mode);
		fl.setNeutralMode(mode);
		rr.setNeutralMode(mode);
		rl.setNeutralMode(mode);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void pidWrite(double output) {
		if(DEBUG)
			Logger.debug("Swerve Drive pidWrite yDirection: " + yDirection + " output=" + output);

		this.set(xDirection, yDirection, output);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void goStraight(double direction, double yaw) {
		this.goStraight(direction, yaw, 0);
	}

	/**
	 * Go straight
	 * @param yDirection forward/backward (-1 to 1)
	 * @param yaw the heading of the robot to maintain
	 * @param xDirection strage (-1 to 1)
	 */
	public void goStraight(double yDirection, double yaw, double xDirection) {
		if(DEBUG)
			Logger.debug("SwerveDrive goStraight direction=" + yDirection + " yaw=" + yaw + " xDirection=" + xDirection);

		this.yDirection = yDirection;
		this.xDirection = xDirection;
		this.setHeading(yaw);
	}
	
	/**
	 * Sets field oriented vs. robot oriented
	 * @param flag true for field oriented, false for robot oriented
	 */
	public void setFieldOriented(boolean flag) {
		if(DEBUG)
			Logger.debug("SwerveDrive setFieldOriented flag=" + flag);

		fieldOriented = flag;
	}

	/**
	 * Tank drive using Swerve. This assumes the wheels 
	 * are fixed straight ahead. This should only be used
	 * in unfortunate situations...
	 * @param left the left output
	 * @param right the right output
	 */
	public void set(double left, double right) {
		fr.setDrive(right);
		fl.setDrive(left);
		rr.setDrive(right);
		rl.setDrive(left);
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
		if(!fieldOriented)
			x = -x;
		
		double fwd = -y;
		double str = x;
		final double rcw = -z;
		
		if(fieldOriented) {
			final double currentAngle = Math.toRadians(this.getAngle()); // make sure to use radians
			final double TEMP = fwd*Math.cos(currentAngle) + str*Math.sin(currentAngle);
			str = -fwd*Math.sin(currentAngle) + str*Math.cos(currentAngle);
			str = -str;
			fwd = TEMP;
		}
		
		final double xNeg = str - (rcw*W_OVER_R);
		final double xPos = str + (rcw*W_OVER_R);
		final double yNeg = fwd - (rcw*L_OVER_R);
		final double yPos = fwd + (rcw*L_OVER_R);
		
		double frSpeed = calcMagnitude(xPos, yPos);
        double frPivot = calcAngle(xPos, yPos);
        
        double flSpeed = calcMagnitude(xPos, yNeg);
        double flPivot = calcAngle(xPos, yNeg);
        
        double rlSpeed = calcMagnitude(xNeg, yNeg);
        double rlPivot = calcAngle(xNeg, yNeg);
        
        double rrSpeed = calcMagnitude(xNeg, yPos);
        double rrPivot = calcAngle(xNeg, yPos);
		
		// Make sure we don't use the arctan value
		// with x=0
		if((xNeg == 0 && Math.abs(fwd) < 0.005) || (Math.abs(x) < 0.005 && Math.abs(y) < 0.005 && Math.abs(z) < 0.005)) {
			frPivot = fr.getAngle();
			flPivot = fl.getAngle();
			rrPivot = rr.getAngle();
			rlPivot = rl.getAngle();
		}
		else if(xNeg == 0) {
			if(fwd > 0.02) {
				frPivot = flPivot = rrPivot = rlPivot = 180;
			}
			else if(fwd < -0.15) {
				frPivot = flPivot = rrPivot = rlPivot = 0;
			}
			else {
				frPivot = fr.getAngle();
				flPivot = fl.getAngle();
				rrPivot = rr.getAngle();
				rlPivot = rl.getAngle();
			}
				
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

		if(DEBUG) {
			Logger.debug("SwerveDrive set x=" + x + " y=" + y + " z=" + z);
			Logger.debug("FL speed=" + flSpeed + " pivot=" + flPivot + " :: FR speed=" + frSpeed + " pivot=" + frPivot);
			Logger.debug("RL speed=" + rlSpeed + " pivot=" + rlPivot + " :: RL speed=" + rrSpeed + " pivot=" + rrPivot);
		}
	}
	
	/**
	 * Sets all modues to the specified output and pivot angle
	 * @param output the speed ranging from 0 to 1
	 * @param angle the angle ranging from 0 to 360
	 */
	public void setAll(double output, double angle) {
		if(DEBUG)
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
		if(DEBUG)
			Logger.debug("SwerveDrive setDrive output=" + output);

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
		if(DEBUG)
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
	public boolean pivotsAtSetpoint() {
		return fr.pivotAtSetpoint() && fl.pivotAtSetpoint() && rr.pivotAtSetpoint() && fl.pivotAtSetpoint();
	}
	
	/**
	 * Stops drive motor on all modules
	 */
	public void stop() {
		super.stop();
		
		fr.stop();
		fl.stop();
		rr.stop();
		rl.stop();

		xDirection = yDirection = 0;
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

		SmartDashboard.putNumber("REMAINING ", REMAINING);
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
		return (Math.toDegrees(Math.atan2(x, -y)) + 360) % 360;
	}
	
	/**
	 * Swerve Module
	 */
	public static class SwerveModule extends SubsystemBase {
		private PIDController pivotPID;
		private double output, offset;
		private boolean flipDrive, reverse, slowOverride;

		private TalonSRX drive, pivot;
		private Encoder driveEnc;
		private AnalogInput pivotEnc;
		private double p, i, d;
		
		// TalonSRX doesn't implement PIDOutput :(
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
		 * @param p the proportional value for the pivot PID
		 * @param i the integral value for the pivot PID
		 * @param d the derivative value for the pivot PID
		 */
		public SwerveModule(TalonSRX drive, Encoder driveEnc, TalonSRX pivot, AnalogInput pivotEnc, double p, double i, double d) {
			this.drive = drive;
			this.driveEnc = driveEnc;
			this.pivot = pivot;
			this.pivotEnc = pivotEnc;
			this.p = p;
			this.i = i;
			this.d = d;
			
			pivotPID = new PIDController(
				p, 
				i, 
				d, 
				pivotEnc, 
				new PIDOutputClass(pivot)
			);
			
			pivotPID.setInputRange(MIN_VOLTAGE, MAX_VOLTAGE);
			pivotPID.setOutputRange(-1, 1);
			pivotPID.setAbsoluteTolerance(toVoltage(TOLERANCE));
			pivotPID.setContinuous();
		}
		
		/**
		 * Sets if the drive motor is overriding elevator height
		 * @param flag true to override, false otherwise
		 */
		public void setSlowOverride(boolean flag) {
			slowOverride = flag;
		}
		
		/**
		 * Sets the drive motor to be reversed
		 * @param flag true to reverse, false otherwise
		 */
		public void setReverse(boolean flag) {
			reverse = flag;
		}

		/**
		 * No default command. Swerve Module only extends SubsystemBase for
		 * convenience reasons
		 */
		@Override
		protected void initDefaultCommand() {
			super.initDefaultCommand();
		}

		/**
		 * Sets the PID values for the internal PID controller for pivot.
		 * @param p the proportional value
		 * @param i the integral value
		 * @param d the derivatve value
		 */
		public void setPID(double p, double i, double d) {
			this.p = p;
			this.i = i;
			this.d = d;
			pivotPID.setPID(p, i, d);
		}

		/**
		 * Sets the P value for the internal PID controller for pivot.
		 * @param p the proportional value
		 */
		public void setP(double p) {
			this.p = p;
			setPID(p, i, d);
		}

		/**
		 * Gets the proportional value for the internal PID controller for pivot.
		 * @return the proportional value
		 */
		public double getP() {
			return p;
		}

		/**
		 * Sets the I value for the internal PID controller for pivot.
		 * @param i the integral value
		 */
		public void setI(double i) {
			this.i = i;
			setPID(p, i, d);
		}

		/**
		 * Gets the integral value for the internal PID controller for pivot.
		 * @return the integral value
		 */
		public double getI() {
			return i;
		}

		/**
		 * Sets the D value for the intenral PID controller for pivot.
		 * @param d the derivative value
		 */
		public void setD(double d) {
			this.d = d;
			setPID(p, i, d);
		}

		/**
		 * Gets the derivative value for the internal PID controller forpivotyaw.
		 * @return the derivative value
		 */
		public double getD() {
			return d;
		}

		/**
		 * Sets the offset for the pivot
		 * @param offset the offset for the pivot
		 */
		public void setOffset(double offset) {
			this.offset = offset;
		}

		/**
		 * Sets the neutral mode for the drive and pivot motor
		 * @see com.ctre.phoenix.motorcontrol.NeutralMode
		*/
		public void setNeutralMode(NeutralMode mode) {
			drive.setNeutralMode(mode);
			pivot.setNeutralMode(mode);
		}
		
		/**
		 * Sets the speed for the drive motor
		 * @param output the speed ranging from -1 to 1
		 */
		protected void setDrive(double output) {
			if(DEBUG)
				Logger.debug("SwerveModule setDrive output=" + (flipDrive ? -output : output) + " flipDrive=" + flipDrive);
			
			double normalizedOutput = output;
			if(Math.abs(output) > 1) {
				normalizedOutput /= normalizedOutput;
			}
			if(output < -1) {
				normalizedOutput *= -1;
			}
			output = normalizedOutput;
			
			double mult = 1.0, elevPosition = OI.getInstance().Elevator.getPosition();
	    	if(slowOverride) {
	    		mult = 1;
	    	}
	    	else if(elevPosition > 1800) {
	    		mult = 0.5;
	    	}
	    	else if(elevPosition > 2150) {
	    		mult = 0.3;
	    	}
	    	else if(elevPosition > 2500) {
	    		mult = 0.2;
	    	}
			
			this.output = flipDrive ? -output : output;
			this.output = reverse ? -this.output : this.output;
			drive.set(ControlMode.PercentOutput, this.output*DRIVE_OUTPUT_LIMITER*mult);
		}

		/**
		 * Gets if the module has flipped its drive output
		 * @return true if the module has flipped its drive output,
		 * false otherwise
		 */
		public boolean isFlipDrive() {
			return flipDrive;
		}
		
		/**
		 * Sets the pivot angle
		 * @param angle the pivot angle ranging from 0 to 360
		 */
		public void setPivot(double angle) {
			angle %= 360;

			// Check if complementary angle is closer
			double dAngle = Math.abs(angle - this.getAngle());
			flipDrive = dAngle >= 90 && dAngle <= 270;

			// if it is closer then use
			// complementary (e.g., add 180)
			if(flipDrive)
				angle += 180;

			if(DEBUG)
				Logger.debug("SwerveModule setPivot flipDrive=" + flipDrive + " angle=" + angle%360);

			pivotPID.setSetpoint(toVoltage(angle));
			pivotPID.enable();
		}

		/**
		 * Gets the current drive output
		 * @return the current drive output
		 */
		public double getDriveOutput() {
			return output;
		}

		/**
		 * Gets the pivot angle in degrees
		 * @return the pivot angle in degrees
		 */
		public double getAngle() {
			return toAngle(getVoltage());
		}

		/**
		 * Gets the voltage of the of the analog input for the pivot
		 * @return the voltage of the of the analog input for the pivot
		 */
		public double getVoltage() {
			return pivotEnc.getAverageVoltage();
		}

		/**
		 * Gets the drive distance
		 * @return the drive distance
		 */
		public double getDistance() {
			return driveEnc.getDistance();
		}
		
		/**
		 * sets the drive distance to 0
		 */
		public void reset() {
			driveEnc.reset();
		}

		/**
		 * Gets the current pivot setpoint
		 * @return the current pivot setpoint
		 */
		public double getSetpoint() {
			return toAngle(pivotPID.getSetpoint());
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
		 * Converts a voltage from the analog input to an angle
		 * @param voltage the voltage
		 * @param offset the offset
		 * @return the angle of the analog input (e.g., pivot angle)
		 */
		private double toAngle(double voltage) {
			return ((360.0 * (voltage - MIN_VOLTAGE) / DELTA_VOLTAGE) + 360.0 - offset) % 360;
		}

		/**
		 * Converts an angle to voltage for the analog input
		 * @param angle the angle in degrees
		 * @return the voltage of the analog input based on the angle
		 */
		private double toVoltage(double angle) {
			angle %= 360;
			angle += 360; // add 360 to ensure formula gives a value from 0 to 5
			return (MIN_VOLTAGE + (DELTA_VOLTAGE*(offset + angle - 360))/360.0);
		}
	}
}
