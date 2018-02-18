package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetSwerveOverride extends InstantCommand {
	private SwerveDrive drive;
	private boolean override;
	
	public SetSwerveOverride(SwerveDrive drive, boolean override) {
		this.drive = drive;
		this.override = override;
		this.requires(drive);
	}
	
	@Override
	protected void execute() {
		drive.fr.setSlowOverride(override);
		drive.fl.setSlowOverride(override);
		drive.rr.setSlowOverride(override);
		drive.rl.setSlowOverride(override);
	}
}
