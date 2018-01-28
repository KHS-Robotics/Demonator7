package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Stops the drive train
 */
public class StopDrive extends InstantCommand 
{
	private DriveTrainBase drive;
	
	/**
	 * Stops the drive
	 * @param drive the drive
	 */
	public StopDrive(DriveTrainBase drive) 
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
