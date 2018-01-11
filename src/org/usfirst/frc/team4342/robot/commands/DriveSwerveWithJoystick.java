package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;

public class DriveSwerveWithJoystick extends CommandBase {
	private Joystick joystick;
	private SwerveDrive drive;
	
	public DriveSwerveWithJoystick(Joystick joystick, SwerveDrive drive) {
		this.joystick = joystick;
		this.drive = drive;
		
		this.requires(drive);
	}

	@Override
	protected void initialize() {
		drive.stopAll();
	}

	@Override
	protected void execute() {
		drive.set(joystick.getX(), joystick.getY(), joystick.getTwist());
	}

	@Override
	protected void end() {
		drive.stopAll();
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}
}
