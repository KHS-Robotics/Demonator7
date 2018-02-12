package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.commands.drive.DriveStraight;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Drive Straight with Swerve
 */
public class DriveStraightSwerve extends DriveStraight {
    private SwerveDrive drive;
    private double speed;
    private boolean x;

    /**
     * Drive Straight with Swerve
     * @param drive the drive
     * @param speed the speed
     * @param distance the distance
     * @param x true if speed is for x-input (strafe), false for y-input (forward/backward)
     */
    public DriveStraightSwerve(SwerveDrive drive, double speed, double distance, boolean x) {
        super(drive, speed, distance);
        this.drive = drive;
        this.speed = speed;
        this.x = x;
    }

    @Override
    protected void initialize() {
        super.initialize();
        drive.goStraight(speed, drive.getHeading(), x);
    }
}
