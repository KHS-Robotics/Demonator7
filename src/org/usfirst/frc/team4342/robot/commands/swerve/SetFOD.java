package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * For testing
 */
public class SetFOD extends InstantCommand {
    private SwerveDrive drive;
    private boolean flag;

    public SetFOD(SwerveDrive drive, boolean flag) {
        this.drive = drive;
        this.flag = flag;

        this.requires(drive);
    }

    @Override
    protected void initialize() {
        drive.setFieldOriented(flag);
    }
}
