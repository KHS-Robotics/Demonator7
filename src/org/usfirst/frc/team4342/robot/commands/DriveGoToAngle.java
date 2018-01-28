package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

/**
 * Command to change the robot's heading
 */
public class DriveGoToAngle extends CommandBase {
	protected final DriveTrainBase drive;
	protected double yaw;
	
	/**
	 * Command to change the robot's heading
	 * @param drive the drive
	 * @param yaw the yaw (-180 to 180)
	 */
	public DriveGoToAngle(DriveTrainBase drive, double yaw) {
		super(3);
		
		this.drive = drive;
		this.yaw = yaw;
		
		this.requires(drive);
	}

	@Override
	protected void initialize() {
		drive.setHeading(yaw);
	}

	@Override
	protected boolean isFinished() {
		return drive.onTarget() || this.isTimedOut();
	}

	@Override
	protected void execute() {}
	@Override
	protected void end() {}
}
