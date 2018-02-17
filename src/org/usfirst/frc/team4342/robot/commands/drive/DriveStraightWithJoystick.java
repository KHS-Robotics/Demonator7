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

	protected Joystick joystick;
	protected XboxController xbox;
	protected final DriveTrainBase drive;
	
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
		drive.goStraight(getYInput(), yaw);
	}

	@Override
	protected void end() {
		drive.stop();
	}

	/**
	 * Gets the y-input of the joystick or xbox controller
	 * @return the y-input of the joystick or xbox controller
	 */
	protected double getYInput() {
		return xbox != null ? xbox.getY(Hand.kLeft) : joystick.getY();
	}
}
