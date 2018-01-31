package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Command to drive straight with a joystick
 */
public class DriveStraightWithJoystick extends CommandBase {
	private Joystick joystick;
	private DriveTrainBase drive;
	private boolean invertY;
	
	private double yaw;
	
	/**
	 * Command to drive straight with a joystick
	 * @param joystick the joystick
	 * @param drive the drive
	 * @param invertY true to invert y input
	 */
	public DriveStraightWithJoystick(Joystick joystick, DriveTrainBase drive, boolean invertY) {
		this.joystick = joystick;
		this.drive = drive;
		this.invertY = invertY;
		
		this.requires(drive);
	}

	/**
	 * Command to drive straight with a joystick
	 * @param joystick the joystick
	 * @param drive the drive
	 */
	public DriveStraightWithJoystick(Joystick joystick, DriveTrainBase drive) {
		this(joystick, drive, false);
	}

	@Override
	protected void initialize() {
		yaw = drive.getHeading();
	}

	@Override
	protected void execute() {
		drive.goStraight(invertY ? -joystick.getY() : joystick.getY(), yaw);
	}

	@Override
	protected void end() {
		drive.stop();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
