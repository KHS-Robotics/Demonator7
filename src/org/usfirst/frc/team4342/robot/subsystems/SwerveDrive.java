package org.usfirst.frc.team4342.robot.subsystems;

import org.usfirst.frc.team4342.robot.OI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Swerve Drive subsystem
 */
public class SwerveDrive extends SubsystemBase {
	private static final double L = 32.3, W = 23.5; // vehicle’s wheelbase and trackwidth
	private static final double R = Math.sqrt((L*L) + (W*W));
	private static final double P = 0.0, I = 0.0, D = 0.0;
	
	private boolean fieldOriented;
	
	public final SwerveModule fr;
	public final SwerveModule fl;
	public final SwerveModule rr;
	public final SwerveModule rl;
	private final AHRS navx;
	
	/**
	 * Creates a new <code>SwerveDrive</code> subsystem
	 * @param fr the front right swerve module
	 * @param fl the front left swerve module
	 * @param rr the rear right swerve module
	 * @param rl the rear left swerve module
	 * @param navx the NavX
	 */
	public SwerveDrive(SwerveModule fr, SwerveModule fl, SwerveModule rr, SwerveModule rl, AHRS navx) {
		this.fr = fr;
		this.fl = fl;
		this.rr = rr;
		this.rl = rl;
		this.navx = navx;
	}
	
	/**
	 * Sets the default command to <code>DriveSwerveWithJoystick</code>
	 */
	@Override
	protected void initDefaultCommand() {
//		OI oi = OI.getInstance();
//		this.setDefaultCommand(new DriveSwerveWithJoystick(oi.DriveStick, oi.SwerveDrive));
	}
	
	/**
	 * Resets the NavX
	 */
	public void resetNavX() {
		navx.reset();
	}
	
	/**
	 * Sets field oriented vs. robot oriented
	 * @param flag true for field oriented, false for robot oriented
	 */
	public void setFieldOriented(boolean flag) {
		fieldOriented = flag;
	}
	
	/**
	 * Sets the x, y and z for the drive train
	 * @param x the x input (e.g., strafe)
	 * @param y the y input (e.g., forward/backward)
	 * @param z the z input (e.g., twist)
	 */
	public void set(double x, double y, double z) {
		double fwd = -y;
		double str = x;
		final double rcw = z;
		
		if(fieldOriented) {
			final double TEMP = fwd*Math.cos(navx.getAngle()) + str*Math.sin(navx.getAngle());
			str = -fwd*Math.sin(navx.getAngle()) + str*Math.cos(navx.getAngle());
			fwd = TEMP;
		}
		
		
		final double A = str - (rcw*(L/R));
		final double B = str + (rcw*(L/R));
		final double C = fwd - (rcw*(W/R));
		final double D = fwd + (rcw*(W/R));
		
		double frSpeed = Math.sqrt((B*B) + (C*C));
		final double frPivot = Math.toDegrees(Math.atan2(B, C));
		
		double flSpeed = Math.sqrt((B*B) + (D*D));
		final double flPivot = Math.toDegrees(Math.atan2(B, D));
		
		double rlSpeed = Math.sqrt((A*A) + (D*D));
		final double rlPivot = Math.toDegrees(Math.atan2(A, D));
		
		double rrSpeed = Math.sqrt((A*A) + (C*C));
		final double rrPivot = Math.toDegrees(Math.atan2(A, C));
		
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
		
		fr.setDrive(frSpeed);
		fr.setPivot(frPivot);
		
		fl.setDrive(flSpeed);
		fl.setPivot(flPivot);
		
		rl.setDrive(rlSpeed);
		rl.setPivot(rlPivot);
		
		rr.setDrive(rrSpeed);
		rr.setPivot(rrPivot);
	}
	
	/**
	 * Sets all modues to the specified output and pivot angle
	 * @param output the speed ranging from 0 to 1
	 * @param pivot the angle ranging from -180 to 180 
	 */
	public void setAll(double output, double pivot) {
		fr.setPivot(pivot);
		fl.setPivot(pivot);
		rr.setPivot(pivot);
		rl.setPivot(pivot);
		
		fr.setDrive(output);
		fl.setDrive(output);
		rr.setDrive(output);
		rl.setDrive(output);
	}
	
	/**
	 * Stops all modules
	 */
	public void stop() {
		fr.stop();
		fl.stop();
		rr.stop();
		rl.stop();
	}
	
	/**
	 * Swerve Module
	 */
	public static class SwerveModule {
		private TalonSRX drive;
		private PIDController pivotPID;
		
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
		 * @param pivot the pivot motor for rotation
		 * @param encoder the encoder for the pivot motor
		 */
		public SwerveModule(TalonSRX drive, TalonSRX pivot, Encoder encoder) {
			this.drive = drive;
			
			pivotPID = new PIDController(P, I, D, encoder, new PIDOutputClass(pivot));
			pivotPID.setInputRange(-180, 180);
			pivotPID.setOutputRange(-1, 1);
			pivotPID.setContinuous();
		}
		
		/**
		 * Sets the drive motor
		 * @param output the speed ranging from 0 to 1
		 */
		public void setDrive(double output) {
			drive.set(ControlMode.PercentOutput, output);
		}
		
		/**
		 * Sets the pivot angle
		 * @param angle the pivot angle ranging from -180 to 180
		 */
		public void setPivot(double angle) {
			pivotPID.enable();
			pivotPID.setSetpoint(angle);
		}
		
		/**
		 * Stops the swerve module
		 */
		public void stop() {
			drive.set(ControlMode.PercentOutput, 0);
			if(pivotPID.isEnabled())
				pivotPID.disable();
		}
	}
}