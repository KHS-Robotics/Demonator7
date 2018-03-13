package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraight;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraightSwerve;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Auto routine to cross the auto line
 */
public class AutoBaseline extends AutonomousRoutine
{
	private final double BASELINE_DISTANCE = 140 - ROBOT_X;

	/**
	 * Auto routine to cross the auto line
	 * @param position the starting position
	 * @param drive the drive
	 * @see StartPosition
	 */
	public AutoBaseline(StartPosition position, SwerveDrive drive) 
	{
		super(position);
		
		this.addSequential(new DriveStraight(drive, -0.8, BASELINE_DISTANCE));
	}
}
