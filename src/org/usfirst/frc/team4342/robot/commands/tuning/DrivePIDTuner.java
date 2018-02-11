package org.usfirst.frc.team4342.robot.commands.tuning;

import org.usfirst.frc.team4342.robot.Constants;
import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to tune drive PID
 */
public class DrivePIDTuner extends PIDTuner {
    private DriveTrainBase drive;

    public DrivePIDTuner(DriveTrainBase drive) {
        this.drive = drive;

        this.requires(drive);

        SmartDashboard.putNumber("Drive-P", SmartDashboard.getNumber("Drive-P", 0.0));
        SmartDashboard.putNumber("Drive-I", SmartDashboard.getNumber("Drive-I", 0.0));
        SmartDashboard.putNumber("Drive-D", SmartDashboard.getNumber("Drive-D", 0.0));
        SmartDashboard.putNumber("Drive-Setpoint", SmartDashboard.getNumber("Drive-Setpoint", 0.0));
    }

    @Override
    protected void setPID() {
        drive.setPID(
            SmartDashboard.getNumber("Drive-P", 0.0),
            SmartDashboard.getNumber("Drive-I", 0.0),
            SmartDashboard.getNumber("Drive-D", 0.0)
        );
    }

    @Override
    protected void setSetpoint() {
        final double heading = drive.getHeading();
        SmartDashboard.putNumber("Drive-Heading", heading);
        drive.setHeading(SmartDashboard.getNumber("Drive-Setpoint", heading));
    }

    @Override
    protected void resetPIDValues() {
        drive.setPID(Constants.Drive.P, Constants.Drive.I, Constants.Drive.D);
    }
}
