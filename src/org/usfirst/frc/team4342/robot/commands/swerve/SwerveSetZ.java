package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.command.Command;

/**
 * For testing
 */
public class SwerveSetZ extends Command {
    private SwerveDrive drive;
    private boolean backwards;

    public SwerveSetZ(SwerveDrive drive, boolean backwards) {
        this.drive = drive;
        this.backwards = backwards;

        this.requires(drive);
    }

    @Override
    protected void initialize() {
        drive.set(0, 0, backwards ? -0.33 : 0.33);
    }
    
    @Override
    protected boolean isFinished() {
    	return false;
    }
}
