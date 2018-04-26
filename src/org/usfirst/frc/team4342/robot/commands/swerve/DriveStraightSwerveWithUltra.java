package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.Constants.Drive;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Drive Straight with Swerve
 */
public class DriveStraightSwerveWithUltra extends DriveStraight {
    private SwerveDrive drive;
    private double xSpeed, ySpeed, ultra;

    /**
     * Drive Straight with Swerve
     * @param drive the drive
     * @param xSpeed the strafe speed (-1 to 1)
     * @param ySpeed the foward/backward speed (-1 to 1)
     * @param distance the distance in inches
     */
    public DriveStraightSwerveWithUltra(SwerveDrive drive, double xSpeed, double ySpeed, double distance, double ultra) {
        super(drive, ySpeed, distance);
        this.drive = drive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.ultra = ultra;
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
    
    protected boolean isFinished()
    {
    	if(((drive.getLeftUltra() + drive.getRightUltra())/2) <= ultra)
    		return true;
    	else
    		return false;
    }
}
