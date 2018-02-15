package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.commands.TeleopCommand;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Drive Swerve With Joystick
 */
public class DriveSwerveWithJoystick extends TeleopCommand {
	private static final double DEADBAND = 0.02;

	private boolean idle;

	private Joystick joystick;
	private SwerveDrive drive;
	
	/**
	 * Drive Swerve With Joystick
	 * @param joystick the joystick to control the swerve drive
	 * @param drive the swerve drive
	 */
	public DriveSwerveWithJoystick(Joystick joystick, SwerveDrive drive) {
		this.joystick = joystick;
		this.drive = drive;
		
		this.requires(drive.fr);
		this.requires(drive.fl);
		this.requires(drive.rr);
		this.requires(drive.rl);
		
		this.requires(drive);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void initialize() {
		drive.stop();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void execute() {
		double xInput = joystick.getX();
		double yInput = joystick.getY();
		double zInput = joystick.getTwist();

		boolean x = Math.abs(xInput) > DEADBAND;
		boolean y = Math.abs(yInput) > DEADBAND;
		boolean z = Math.abs(zInput) > DEADBAND+0.03;

		xInput = x ? xInput : 0;
		yInput = y ? yInput : 0;
		zInput = z ? zInput : 0;

		if(x || y || z) {
			drive.set(xInput, yInput, zInput);
			idle = false;
		}
		else if(!idle) {
			drive.stop();
			idle = true;
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void end() {
		drive.stop();
	}
}
