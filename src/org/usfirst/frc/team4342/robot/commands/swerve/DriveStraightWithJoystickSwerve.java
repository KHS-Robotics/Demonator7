package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.commands.drive.DriveStraightWithJoystick;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Drive Straight with Joystick with Swerve
 */
public class DriveStraightWithJoystickSwerve extends DriveStraightWithJoystick {
    private SwerveDrive drive;

    public DriveStraightWithJoystickSwerve(XboxController xbox, SwerveDrive drive) {
        super(xbox, drive);
        this.drive = drive;
    }
    
    public DriveStraightWithJoystickSwerve(Joystick joystick, SwerveDrive drive) {
        super(joystick, drive);
        this.drive = drive;
	}
    
    @Override
    protected void execute() {
        drive.goStraight(getYInput(), yaw, getXInput());
    }

    /**
	 * Gets the x-input of the joystick or xbox controller
	 * @return the x-input of the joystick or xbox controller
	 */
    protected double getXInput() {
        return xbox != null ? xbox.getX(Hand.kRight) : joystick.getX();
    }
}
