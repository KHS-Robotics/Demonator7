package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

/**
 * Command to drive straight
 */
public class TankDriveStraightDistance extends CommandBase {
	private TankDrive drive;
	private double speed, yaw, distance;

	private double initialLeft, initialRight;
	
	/**
	 * Command to drive straight at a certain yaw
	 * @param drive the drive
	 * @param speed the desired speed (-1 to 1)
	 * @param yaw the yaw the robot's heading should be facing (-180 to 180)
	 * @param distance the distance to move in inches
	 */
	public TankDriveStraightDistance(TankDrive drive, double speed, double yaw, double distance) {
		this.drive = drive;
		this.speed = speed;
		this.yaw = yaw;
		this.distance = distance;
		
		this.requires(drive);
	}

	/**
	 * Command to drive straight at the robot's current heading
	 * @param drive the drive
	 * @param speed the desired speed from -1 to 1
	 * @param distance the distance to move
	 */
	public TankDriveStraightDistance(TankDrive drive, double speed, double distance) {
		this(drive, speed, Double.MAX_VALUE, distance);
	}
	
	@Override
	protected void initialize() {
		if(yaw == Double.MAX_VALUE) {
			yaw = drive.getHeading();
		}

		initialLeft = drive.getLeftDistance();
		initialRight = drive.getRightDistance();
		drive.goStraight(speed, yaw);
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
