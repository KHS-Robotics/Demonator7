package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Command to drive straight
 */
public class SwerveGoStraightDistance extends CommandBase {
    private double initalFR, initalFL, initalRR, initalRL;
    private boolean atSetpoint;

    private SwerveDrive drive;
    private double distance, speed, angle;

    /**
     * Command to drive straight
     * @param drive the swerve drive
     * @param distance the distance to move in inches
     * @param speed the desired speed ranging from -1 to 1
     * @param angle the angle of the pivot motors for the swerve modules ranging from 0 to 360
     */
    public SwerveGoStraightDistance(SwerveDrive drive, double distance, double speed, double angle) {
        this.drive = drive;
        this.distance = distance;
        this.speed = speed;
        this.angle = angle;

        this.requires(drive);
    }

    @Override
    protected void initialize() {
        drive.stop();
        
        drive.setPivot(angle);

        initalFR = drive.getFRDistance();
        initalFL = drive.getFLDistance();
        initalRR = drive.getRRDistance();
        initalRL = drive.getRLDistance();
    }

    @Override
    protected void execute() {
        if(!atSetpoint && drive.pivotAtSetpoint()) {
            atSetpoint = true;
            drive.setDrive(speed);
        }
    }

    @Override
    protected void end() {
        drive.stop();
    }

    @Override
    protected boolean isFinished() {
        return drive.remainingDistance(distance, initalFR, initalFL, initalRR, initalRL) <= 0;
    }
}
