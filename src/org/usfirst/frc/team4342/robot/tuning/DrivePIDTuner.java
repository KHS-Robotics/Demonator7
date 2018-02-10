package org.usfirst.frc.team4342.robot.tuning;

import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to tune drive PID
 */
public class DrivePIDTuner extends Thread implements Runnable {
    private DriveTrainBase drive;

    public DrivePIDTuner(DriveTrainBase drive) {
        this.drive = drive;

        SmartDashboard.putNumber("Drive-P", 0.0);
        SmartDashboard.putNumber("Drive-I", 0.0);
        SmartDashboard.putNumber("Drive-D", 0.0);
    }
    
    @Override
    public void run() {
        while(!Thread.interrupted()) {
            drive.setPID(
                SmartDashboard.getNumber("Drive-P", 0.0),
                SmartDashboard.getNumber("Drive-I", 0.0),
                SmartDashboard.getNumber("Drive-D", 0.0)
            );

            SmartDashboard.putNumber("Drive-Heading", drive.getHeading());
        }

        drive.setPID(0, 0, 0);
    }
}
