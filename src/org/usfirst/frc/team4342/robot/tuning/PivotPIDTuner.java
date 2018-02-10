package org.usfirst.frc.team4342.robot.tuning;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to tune pivot PID for swerve
 */
public class PivotPIDTuner extends Thread implements Runnable {
    private SwerveModule module;
    private String name;

    public PivotPIDTuner(SwerveModule module, String name) {
        this.module = module;
        this.name = name;

        SmartDashboard.putNumber(name + "-Pivot-P", 0.0);
        SmartDashboard.putNumber(name + "-Pivot-I", 0.0);
        SmartDashboard.putNumber(name + "-Pivot-D", 0.0);
    }
    
    @Override
    public void run() {
        while(!Thread.interrupted()) {
            module.setPID(
                SmartDashboard.getNumber(name + "-Pivot-P", 0.0),
                SmartDashboard.getNumber(name + "-Pivot-I", 0.0),
                SmartDashboard.getNumber(name + "-Pivot-D", 0.0)
            );

            SmartDashboard.putNumber(name + "-Pivot-Angle", module.getAngle());
        }

        module.setPID(0, 0, 0);
    }
}
