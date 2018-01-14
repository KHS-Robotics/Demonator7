package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopTankDrive extends InstantCommand {
	private TankDrive drive;
	
	public StopTankDrive(TankDrive drive) {
		this.drive = drive;
		
		this.requires(drive);
	}
	
	@Override
	protected void execute() {
		drive.stop();
	}
}
