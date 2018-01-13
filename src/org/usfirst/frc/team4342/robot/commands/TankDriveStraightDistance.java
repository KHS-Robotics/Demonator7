package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

public class TankDriveStraightDistance extends CommandBase {
	private TankDrive drive;
	private double direction, yaw, distance;

	private double initialLeft, initialRight;
	
	public TankDriveStraightDistance(TankDrive drive, double direction, double yaw, double distance) {
		this.drive = drive;
		this.direction = direction;
		this.yaw = yaw;
		this.distance = distance;
		
		this.requires(drive);
	}
	
	@Override
	protected void initialize() {
		initialLeft = drive.getLeftDistance();
		initialRight = drive.getRightDistance();
		drive.goStraight(direction, yaw);
	}

	@Override
	protected boolean isFinished() {
		return drive.remainingDistance(distance, initialLeft, initialRight) <= 0;
	}

	@Override
	protected void end() {
		drive.disablePID();
		drive.set(0, 0);
	}

	@Override
	protected void execute() {}
}
