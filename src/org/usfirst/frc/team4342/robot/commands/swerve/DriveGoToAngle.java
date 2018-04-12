package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.commands.CommandBase;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Command to change the robot's heading
 */
public class DriveGoToAngle extends CommandBase {
	protected final SwerveDrive drive;
	protected double yaw;
	
	/**
	 * Command to change the robot's heading
	 * @param drive the drive
	 * @param yaw the yaw (-180 to 180)
	 */
	public DriveGoToAngle(SwerveDrive drive, double yaw) {
		super(2.5);
		
		this.drive = drive;
		this.yaw = yaw;
		
		this.requires(drive);
	}

	@Override
	protected void initialize() {
		drive.setHeading(yaw);
	}

	@Override
	protected void end() {
		drive.stop();
	}

	@Override
	protected boolean isFinished() {
		return drive.onTarget() || this.isTimedOut();
	}

	@Override
	protected void execute() {}
}
