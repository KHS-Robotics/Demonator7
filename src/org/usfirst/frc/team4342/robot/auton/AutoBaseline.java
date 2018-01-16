package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.TankDriveStraightDistance;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

/**
 * Auto routine to cross the auto line
 */
public class AutoBaseline extends AutonomousRoutine
{
	/**
	 * Auto routine to cross the auto line
	 * @param position the starting position
	 * @param drive the tank drive
	 * @see StartPosition
	 */
	public AutoBaseline(StartPosition position, TankDrive drive) 
	{
		super(position);
		
		this.addSequential(new TankDriveStraightDistance(drive, 0.5, 0, BASELINE_DISTANCE));
	}

	/**
	 * Auto routine to cross the auto line
	 * @param drive the tank drive
	 */
	public AutoBaseline(TankDrive drive)
	{
		this(StartPosition.CENTER, drive);
	}
}
