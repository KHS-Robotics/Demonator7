package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Drive Straight with Swerve
 */
public class DriveStraightSwerve extends DriveStraight {
    private SwerveDrive drive;
    private double xSpeed, ySpeed;

    /**
     * Drive Straight with Swerve
     * @param drive the drive
     * @param xSpeed the strafe speed (-1 to 1)
     * @param ySpeed the foward/backward speed (-1 to 1)
     * @param distance the distance in inches
     */
    public DriveStraightSwerve(SwerveDrive drive, double xSpeed, double ySpeed, double distance) {
        super(drive, ySpeed, distance);
        this.drive = drive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

    @Override
    protected void initialize() {
        super.initialize();
        drive.fl.reset();
        drive.fr.reset();
        drive.rl.reset();
        drive.rr.reset();
        drive.goStraight(ySpeed, drive.getHeading(), xSpeed);
    }
}
