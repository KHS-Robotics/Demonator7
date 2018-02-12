package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.drive.DriveStraight;
import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;

/**
 * Auto routine to cross the auto line
 */
public class AutoBaseline extends AutonomousRoutine
{
	private final double BASELINE_DISTANCE = 120 - ROBOT_Y;

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
}
