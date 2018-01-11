package org.usfirst.frc.team4342.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

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
	
	public SwerveDrive(SwerveModule fr, SwerveModule fl, SwerveModule rr, SwerveModule rl, AHRS navx) {
		this.fr = fr;
		this.fl = fl;
		this.rr = rr;
		this.rl = rl;
		this.navx = navx;
	}
	
	@Override
	protected void initDefaultCommand() {
		// TODO: Set default command to DriveSwerveWithJoystick
	}
	
	public void resetAngle() {
		navx.reset();
	}
	
	public void setFieldOriented(boolean flag) {
		fieldOriented = flag;
	}
	
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
	
	public void stopAll() {
		fr.stop();
		fl.stop();
		rr.stop();
		rl.stop();
	}
	
	public class SwerveModule {
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
		
		public SwerveModule(TalonSRX drive, TalonSRX pivot, Encoder encoder) {
			this.drive = drive;
			
			pivotPID = new PIDController(P, I, D, encoder, new PIDOutputClass(pivot));
			pivotPID.setInputRange(-180, 180);
			pivotPID.setOutputRange(-1, 1);
			pivotPID.setContinuous();
		}
		
		public void setDrive(double output) {
			drive.set(ControlMode.PercentOutput, output);
		}
		
		public void setPivot(double angle) {
			pivotPID.enable();
			pivotPID.setSetpoint(angle);
		}
		
		public void stop() {
			drive.set(ControlMode.PercentOutput, 0);
			pivotPID.disable();
		}
	}
}
