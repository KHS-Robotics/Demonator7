package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.swerve.DriveGoToAngle;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraight;
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
public class AutoSwitch extends AutonomousRoutine 
{	
	// Center Switch
	private static final double CENTER_DIAGONAL_DISTANCE = 169.8 + ROBOT_Y;
	// Left or Right Switch
	// Start Position and Switch location are the same
	private static final double LEFT_RIGHT_PANEL_ALIGN_DISTANCE = 168;
	private static final double LEFT_RIGHT_SWITCH_DISTANCE = 60 - (ROBOT_X / 2);
	// Start Position and Switch location are opposite
	private static final double LEFT_RIGHT_PAST_SWITCH_DISTANCE = 210;
	private static final double LEFT_RIGHT_PAST_SWITCH_ALIGN_DISTANCE = 180 - (ROBOT_X/2);
	private static final double LEFT_RIGHT_MOVE_TO_SWITCH_DISTANCE = 18;

	/**
	 * Auto routine to place a cube on the switch for the
	 * specified position
	 * @param position the starting position
	 * @param d the drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoSwitch(StartPosition position, SwerveDrive d, Elevator e, Intake i) 
	{
		super(position);
		
		try 
		{
			if(position == StartPosition.LEFT)
			{
				// Starting turned 90 deg clockwise
				d.setHeadingOffset(90);

				if(isSwitchLeft())
				{
					this.addParallel(new ElevateToSwitch(e));
					this.addSequential(new DriveStraightSwerve(d, -0.5, 0.0, LEFT_RIGHT_PANEL_ALIGN_DISTANCE));
					this.addSequential(new DriveStraight(d, 0.5, LEFT_RIGHT_SWITCH_DISTANCE));
					this.addSequential(new ReleaseCube(i));
					// TODO: Pick up another cube and put in on our switch plate or scale
				}
				else
				{
					this.addSequential(new DriveStraightSwerve(d, -0.5, 0.0, LEFT_RIGHT_PAST_SWITCH_DISTANCE));
					this.addParallel(new ElevateToSwitch(e));
					this.addSequential(new DriveStraight(d, 0.5, LEFT_RIGHT_PAST_SWITCH_ALIGN_DISTANCE));
					this.addSequential(new DriveGoToAngle(d, 180));
					this.addSequential(new DriveStraight(d, 0.5, LEFT_RIGHT_MOVE_TO_SWITCH_DISTANCE));
					this.addSequential(new ReleaseCube(i));
					// TODO: Pick up another cube and put in on our switch plate or scale
				}
					
			}
			else if(position == StartPosition.RIGHT)
			{
				// Starting turned 90 deg counterclockwise
				d.setHeadingOffset(-90);

				if(isSwitchRight())
				{
					this.addParallel(new ElevateToSwitch(e));
					this.addSequential(new DriveStraightSwerve(d, 0.5, 0.0, LEFT_RIGHT_PANEL_ALIGN_DISTANCE));
					this.addSequential(new DriveStraight(d, 0.5, LEFT_RIGHT_SWITCH_DISTANCE));
					this.addSequential(new ReleaseCube(i));
					// TODO: Pick up another cube and put in on our switch plate or scale
				}
				else
				{
					this.addSequential(new DriveStraightSwerve(d, 0.5, 0.0, LEFT_RIGHT_PAST_SWITCH_DISTANCE));
					this.addParallel(new ElevateToSwitch(e));
					this.addSequential(new DriveStraight(d, 0.5, LEFT_RIGHT_PAST_SWITCH_ALIGN_DISTANCE));
					this.addSequential(new DriveGoToAngle(d, 180));
					this.addSequential(new DriveStraight(d, 0.5, LEFT_RIGHT_MOVE_TO_SWITCH_DISTANCE));
					this.addSequential(new ReleaseCube(i));
					// TODO: Pick up another cube and put in on our switch plate or scale
				}
			}
			else if(position == StartPosition.CENTER)
			{
				final double xSpeed = isSwitchRight() ? 0.5 : -0.5;
				
				this.addParallel(new ElevateToSwitch(e));
				this.addSequential(new DriveStraightSwerve(d, xSpeed, 0.5, CENTER_DIAGONAL_DISTANCE));
				this.addSequential(new ReleaseCube(i));
				// TODO: Pick up another cube and put in on our switch plate
			}
			else
			{
				Logger.warning("No position provided for AutoSwitch! Crossing Baseline...");
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
