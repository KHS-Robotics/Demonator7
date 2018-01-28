package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Command to drive straight with a joystick
 */
public class DriveStraightWithJoystick extends CommandBase {
	private Joystick joystick;
	private DriveTrainBase drive;
	
	private double yaw;
	
	/**
	 * Command to drive straight with a joystick
	 * @param joystick the joystick
	 * @param drive the drive
	 */
	public DriveStraightWithJoystick(Joystick joystick, DriveTrainBase drive) {
		this.joystick = joystick;
		this.drive = drive;
		
		this.requires(drive);
	}

	@Override
	protected void initialize() {
		yaw = drive.getHeading();
	}

	@Override
	protected void execute() {
		drive.goStraight(-joystick.getY(), yaw);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {}
}
