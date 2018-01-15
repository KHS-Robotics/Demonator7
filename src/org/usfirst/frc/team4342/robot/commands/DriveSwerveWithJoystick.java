package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Drive Swerve With Joystick
 */
public class DriveSwerveWithJoystick extends CommandBase {
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
		
		this.requires(drive);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void initialize() {
		drive.stopAll();
	}

	@Override
	protected void execute() {
		drive.set(joystick.getX(), joystick.getY(), joystick.getTwist());
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void end() {
		drive.stopAll();
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}
}
