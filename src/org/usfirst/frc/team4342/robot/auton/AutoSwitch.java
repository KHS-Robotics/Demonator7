package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraightDistance;
import org.usfirst.frc.team4342.robot.commands.TurnTank;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

/**
 * Auto routine to place a cube on the switch for the
 * specified position
 * @see StartPosition
 */
public class AutoSwitch extends AutonomousRoutine 
{	
	/**
	 * Auto routine to place a cube on the switch for the
	 * specified position
	 * @param position the starting position
	 * @param d the tank drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoSwitch(StartPosition position, TankDrive d, Elevator e, Intake i) 
	{
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(this.isSwitchLeft())
			{
				this.addParallel(new ElevateToSwitch(e));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_PANEL_ALIGN_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_SWITCH_DISTANCE));
				this.addSequential(new ReleaseCube(i));	
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_PAST_SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addParallel(new ElevateToSwitch(e));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_PAST_SWITCH_ALIGN_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_MOVE_TO_SWITCH_DISTANCE));
				this.addSequential(new ReleaseCube(i));
			}
				
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isSwitchRight())
			{
				this.addParallel(new ElevateToSwitch(e));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_PANEL_ALIGN_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_SWITCH_DISTANCE));
				this.addSequential(new ReleaseCube(i));
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_PAST_SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addParallel(new ElevateToSwitch(e));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_PAST_SWITCH_ALIGN_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_RIGHT_MOVE_TO_SWITCH_DISTANCE));
				this.addSequential(new ReleaseCube(i));
			}
		}
		else if(position == StartPosition.CENTER)
		{
			final boolean clockwise = this.isSwitchRight();
			
			this.addParallel(new ElevateToSwitch(e));
			this.addSequential(new TankDriveStraightDistance(d, 0.5, CENTER_STRAIGHT_DISTANCE));
			this.addSequential(new TurnTank(d, clockwise));
			this.addSequential(new TankDriveStraightDistance(d, 0.5, CENTER_PANEL_ALIGN_DISTANCE));
			this.addSequential(new TurnTank(d, !clockwise));
			this.addSequential(new TankDriveStraightDistance(d, 0.5, CENTER_STRAIGHT_DISTANCE));
			this.addSequential(new ReleaseCube(i));
		}
		else
		{
			Logger.warning("No Position for Auto: Crossing Baseline");
			this.addSequential(new AutoBaseline(position, d));
		}
	}
}
