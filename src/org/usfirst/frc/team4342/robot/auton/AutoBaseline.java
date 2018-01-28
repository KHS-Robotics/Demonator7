package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.DriveStraight;
import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

/**
 * Auto routine to cross the auto line
 */
public class AutoBaseline extends AutonomousRoutine
{
	/**
	 * Auto routine to cross the auto line
	 * @param position the starting position
	 * @param drive the drive
	 * @see StartPosition
	 */
	public AutoBaseline(StartPosition position, DriveTrainBase drive) 
	{
		super(position);
		
		this.addSequential(new DriveStraight(drive, 0.5, BASELINE_DISTANCE));
	}

	/**
	 * Auto routine to cross the auto line
	 * @param drive the drive
	 */
	public AutoBaseline(DriveTrainBase drive)
	{
		this(StartPosition.CENTER, drive);
	}
}
