package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Drive Straight with Joystick with Swerve
 */
public class DriveStraightWithJoystickSwerve extends DriveStraightWithJoystick {
    private SwerveDrive drive;

    public DriveStraightWithJoystickSwerve(XboxController xbox, SwerveDrive drive, boolean x) {
        super(xbox, drive, x);
        
        this.requires(drive.fr);
		this.requires(drive.fl);
		this.requires(drive.rr);
		this.requires(drive.rl);
		
        this.drive = drive;
    }
    
    public DriveStraightWithJoystickSwerve(Joystick joystick, SwerveDrive drive, boolean x) {
        super(joystick, drive, x);
        this.drive = drive;
	}
    
    @Override
    protected void execute() {
        drive.goStaight(getInput(), yaw, isXDirection);
    }
}
