package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Command to drive straight with a joystick
 */
public class DriveStraightWithJoystick extends TeleopCommand {
	private double input;
	
	private Joystick joystick;
	private XboxController xbox;
	private DriveTrainBase drive;
	
	private double yaw;
	
	/**
	 * Command to drive straight with a joystick
	 * @param joystick the joystick
	 * @param drive the drive
	 * @param invertY true to invert y input
	 */
	public DriveStraightWithJoystick(Joystick joystick, DriveTrainBase drive) {
		this.joystick = joystick;
		this.drive = drive;
		
		this.requires(drive);
	}

	public DriveStraightWithJoystick(XboxController xbox, DriveTrainBase drive) {
		this.xbox = xbox;
		this.drive = drive;
		
		this.requires(drive);
	}

	@Override
	protected void initialize() {
		yaw = drive.getHeading();
	}

	@Override
	protected void execute() {
		input = xbox != null ? -xbox.getRawAxis(1) : -joystick.getY();
		drive.goStraight(input, yaw);
	}

	@Override
	protected void end() {
		drive.stop();
	}
}
