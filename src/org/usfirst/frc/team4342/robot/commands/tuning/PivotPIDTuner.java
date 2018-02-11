package org.usfirst.frc.team4342.robot.commands.tuning;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to tune pivot PID for swerve
 */
public class PivotPIDTuner extends PIDTuner {
    private SwerveModule module;
    private String name;
    private double p, i, d;

    public PivotPIDTuner(SwerveModule module, String name, double p, double i, double d) {
        this.module = module;
        this.name = name;
        this.p = p;
        this.i = i;
        this.d = d;
        
        this.requires(module);

        SmartDashboard.putNumber(name + "-Pivot-P", SmartDashboard.getNumber(name + "-Pivot-P", 0.0));
        SmartDashboard.putNumber(name + "-Pivot-I", SmartDashboard.getNumber(name + "-Pivot-I", 0.0));
        SmartDashboard.putNumber(name + "-Pivot-D", SmartDashboard.getNumber(name + "-Pivot-D", 0.0));
        SmartDashboard.putNumber(name + "-Pivot-Setpoint", SmartDashboard.getNumber(name + "-Pivot-Setpoint", 0.0));
    }

    @Override
    protected void setPID() {
        module.setPID(
            SmartDashboard.getNumber(name + "-Pivot-P", 0.0),
            SmartDashboard.getNumber(name + "-Pivot-I", 0.0),
            SmartDashboard.getNumber(name + "-Pivot-D", 0.0)
        );
    }

    @Override
    protected void setSetpoint() {
        final double angle = module.getAngle();
        SmartDashboard.putNumber(name + "-Pivot-Angle", angle);
        module.setPivot(SmartDashboard.getNumber(name + "-Pivot-Setpoint", angle));
    }

    @Override
    protected void resetPIDValues() {
        module.setPID(p, i, d);
    }
}
