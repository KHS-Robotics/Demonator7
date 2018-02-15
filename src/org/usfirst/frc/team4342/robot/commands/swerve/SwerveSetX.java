package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.command.Command;

/**
 * For testing
 */
public class SwerveSetX extends Command {
    private SwerveDrive drive;
    private boolean backwards;

    public SwerveSetX(SwerveDrive drive, boolean backwards) {
        this.drive = drive;
        this.backwards = backwards;

        this.requires(drive);
    }

    @Override
    protected void initialize() {
        drive.set(backwards ? -0.33 : 0.33, 0, 0);
    }
    
    @Override
    protected boolean isFinished() {
    	return false;
    }
}
