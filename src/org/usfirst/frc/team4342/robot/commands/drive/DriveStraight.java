package org.usfirst.frc.team4342.robot.commands.drive;

import org.usfirst.frc.team4342.robot.commands.CommandBase;
import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

/**
 * Command to drive straight
 */
public class DriveStraight extends CommandBase {
    private double[] distances;

    private DriveTrainBase drive;
    private double speed;
    private double distance;

    /**
     * Command to drive straight
     * @param drive the drive
     * @param speed the speed
     * @param distance the distance
     */
    public DriveStraight(DriveTrainBase drive, double speed, double distance) {
        super(10);
        
        this.drive = drive;
        this.speed = speed;
        this.distance = distance;
        
        this.requires(drive);
    }

    @Override
    protected void initialize() {
        distances = drive.getAllDistances();
        drive.goStraight(speed, drive.getHeading());
    }

    @Override
    protected void end() {
        drive.stop();
    }

    @Override
    protected boolean isFinished() {
        return drive.remainingDistance(distance, distances) <= 0;
    }

    @Override
    protected void execute() {}
}
