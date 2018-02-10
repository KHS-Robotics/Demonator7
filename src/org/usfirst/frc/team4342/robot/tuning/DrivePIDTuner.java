package org.usfirst.frc.team4342.robot.tuning;

import org.usfirst.frc.team4342.robot.Constants;
import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to tune drive PID
 */
public class DrivePIDTuner extends Thread implements Runnable {
    private DriveTrainBase drive;

    public DrivePIDTuner(DriveTrainBase drive) {
        this.drive = drive;

        SmartDashboard.putNumber("Drive-P", SmartDashboard.getNumber("Drive-P", 0.0));
        SmartDashboard.putNumber("Drive-I", SmartDashboard.getNumber("Drive-I", 0.0));
        SmartDashboard.putNumber("Drive-D", SmartDashboard.getNumber("Drive-D", 0.0));
        SmartDashboard.putNumber("Drive-Setpoint", SmartDashboard.getNumber("Drive-Setpoint", 0.0));
    }
    
    @Override
    public void run() {
        while(!Thread.interrupted()) {
            final double heading = drive.getHeading();

            if(RobotState.isTest()) {
                drive.setPID(
                    SmartDashboard.getNumber("Drive-P", 0.0),
                    SmartDashboard.getNumber("Drive-I", 0.0),
                    SmartDashboard.getNumber("Drive-D", 0.0)
                );

                drive.setHeading(SmartDashboard.getNumber("Drive-Setpoint", heading));
            }
            
            SmartDashboard.putNumber("Drive-Heading", heading);

            try {
                Thread.sleep(20);
            } catch(InterruptedException ex) {
                drive.setPID(Constants.Drive.P, Constants.Drive.I, Constants.Drive.D);
            }
        }

        drive.setPID(Constants.Drive.P, Constants.Drive.I, Constants.Drive.D);
    }
}
