package org.usfirst.frc.team4342.robot.commands.drive;

import org.usfirst.frc.team4342.robot.commands.TeleopCommand;
import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Command to drive straight with a joystick
 */
public class DriveStraightWithJoystick extends TeleopCommand {
	protected double yaw;

	private Joystick joystick;
	private XboxController xbox;
	private DriveTrainBase drive;
	protected boolean isXDirection;
	
	/**
	 * Command to drive straight with a joystick
	 * @param joystick the joystick
	 * @param drive the drive
	 * @param x true to use go straight with x, false to use y
	 */
	public DriveStraightWithJoystick(Joystick joystick, DriveTrainBase drive, boolean x) {
		this.joystick = joystick;
		this.drive = drive;
		this.isXDirection = x;
		
		this.requires(drive);
	}

	public DriveStraightWithJoystick(Joystick joystick, DriveTrainBase drive) {
		this(joystick, drive, false);
	}

	public DriveStraightWithJoystick(XboxController xbox, DriveTrainBase drive, boolean x) {
		this.xbox = xbox;
		this.drive = drive;
		this.isXDirection = x;
		
		this.requires(drive);
	}

	public DriveStraightWithJoystick(XboxController xbox, DriveTrainBase drive) {
		this(xbox, drive, false);
	}

	@Override
	protected void initialize() {
		yaw = drive.getHeading();
	}

	@Override
	protected void execute() {
		drive.goStraight(getInput(), yaw);
	}

	@Override
	protected void end() {
		drive.stop();
	}

	/**
	 * Gets the input of the joystick or xbox controller
	 * @return the input of the joystick or xbox controller
	 */
	protected double getInput() {
		if(isXDirection) {
			return xbox != null ? xbox.getX(Hand.kRight) : joystick.getX();
		}
		return xbox != null ? -xbox.getY(Hand.kLeft) : -joystick.getY();
	}
}
