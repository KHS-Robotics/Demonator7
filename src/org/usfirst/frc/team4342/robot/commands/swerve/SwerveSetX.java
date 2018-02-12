package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * For testing
 */
public class SwerveSetX extends InstantCommand {
    private SwerveDrive drive;
    private boolean backwards;

    public SwerveSetX(SwerveDrive drive, boolean backwards) {
        this.drive = drive;
        this.backwards = backwards;

        this.requires(drive);
    }

    @Override
    protected void execute() {
        drive.set(backwards ? -0.20 : 0.20, 0, 0);
    }
}
