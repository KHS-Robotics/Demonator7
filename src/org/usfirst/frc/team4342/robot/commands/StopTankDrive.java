package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Stops the tank drive
 */
public class StopTankDrive extends InstantCommand 
{
	private TankDrive drive;
	
	/**
	 * Stops the tank drive
	 * @param drive the tank drive
	 */
	public StopTankDrive(TankDrive drive) 
	{
		this.drive = drive;
		
		this.requires(drive);
	}
	
	@Override
	protected void execute() 
	{
		drive.stop();
	}
}
