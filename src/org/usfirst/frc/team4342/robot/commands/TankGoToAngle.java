package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

public class TankGoToAngle extends CommandBase {
	protected final TankDrive drive;
	private double yaw;
	
	public TankGoToAngle(TankDrive drive, double yaw) {
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

	protected void setSetpoint(double yaw) {
		this.yaw = yaw;
	}

	protected double getSetpoint() {
		return yaw;
	}

	@Override
	protected void execute() {}
	@Override
	protected void end() {}
}
