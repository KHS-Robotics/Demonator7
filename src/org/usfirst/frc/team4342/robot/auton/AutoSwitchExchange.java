package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.intake.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraightSwerve;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Auto routine to place a cube on the switch for the specified position
 */
public class AutoSwitchExchange extends AutonomousRoutine 
{	
	// Center Switch
	private static final double CENTER_DIAGONAL_DISTANCE = 88 + ROBOT_Y;

	/**
	 * Auto routine to place a cube on the switch for the
	 * specified position
	 * @param position the starting position
	 * @param d the drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoSwitchExchange(StartPosition position, SwerveDrive d, Elevator e, Intake i) 
	{
		super(position);
		
		try 
		{
			
			if(position == StartPosition.CENTER) 
			{
				final double xSpeed = isSwitchRight() ? 0.3 : -0.35;
				final double offset = isSwitchRight() ? -7 : 0;
				this.addParallel(new ElevateToSwitch(e));
				this.addSequential(new DriveStraightSwerve(d, xSpeed, -0.6, CENTER_DIAGONAL_DISTANCE + offset));
				this.addSequential(new ReleaseCube(i));
				// TODO: Pick up another cube and put in on our switch plate
			}
			else
			{
				Logger.warning("Invalid position provided for AutoSwitch! Crossing Baseline...");
				this.addSequential(new AutoBaseline(position, d));
			}
		}
		catch(InvalidGameMessageException ex)
		{
			Logger.error(ex.getMessage(), ex);
			Logger.warning("Invalid Game Message! Corssing Baseline...");
			this.addSequential(new AutoBaseline(position, d));
		}
	}
}
