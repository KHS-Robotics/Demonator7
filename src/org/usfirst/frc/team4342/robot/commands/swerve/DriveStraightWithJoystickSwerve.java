package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.commands.TeleopCommand;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Command to drive straight with a joystick
 */
public class DriveStraightWithJoystickSwerve extends TeleopCommand {
	protected double yaw;

	protected Joystick joystick;
	protected XboxController xbox;
	protected final SwerveDrive drive;
	
	/**
	 * Command to drive straight with a joystick
	 * @param joystick the joystick
	 * @param drive the drive
	 */
	public DriveStraightWithJoystickSwerve(Joystick joystick, SwerveDrive drive) {
		this.joystick = joystick;
		this.drive = drive;
		
		this.requires(drive);
	}

	public DriveStraightWithJoystickSwerve(XboxController xbox, SwerveDrive drive) {
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
		drive.goStraight(getYInput(), yaw, getXInput());
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
	
	/**
	 * Gets the x-input of the joystick or xbox controller
	 * @return the x-input of the joystick or xbox controller
	 */
    protected double getXInput() {
        return xbox != null ? xbox.getX(Hand.kRight) : joystick.getX();
    }
}
