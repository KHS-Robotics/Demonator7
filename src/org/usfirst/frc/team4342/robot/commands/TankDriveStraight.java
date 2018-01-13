package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

import edu.wpi.first.wpilibj.Joystick;

public class TankDriveStraight extends CommandBase {
	private Joystick joystick;
	private TankDrive drive;
	
	private double yaw;
	
	public TankDriveStraight(Joystick joystick, TankDrive drive) {
		this.joystick = joystick;
		this.drive = drive;
		
		this.requires(drive);
	}

	@Override
	protected void initialize() {
		yaw = drive.getHeading();
	}

	@Override
	protected void execute() {
		drive.goStraight(joystick.getY(), yaw);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {}
	
}
