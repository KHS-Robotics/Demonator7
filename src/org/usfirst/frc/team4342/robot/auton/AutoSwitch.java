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
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));	
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, PAST_SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 185.5)); //distance from expected position to other plate
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, PAST_SWITCH_DISTANCE - 261.47)); //261.47 is far wall of switch
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));
			}
				
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isSwitchRight())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, PAST_SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 185.5)); //distance from expected position to other plate
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, PAST_SWITCH_DISTANCE - 261.47)); //261.47 is far wall of switch
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));
			}
		}
		else if(position == StartPosition.CENTER)
		{
			/**
			 * 36 is to avoid the exchange, 84.75 distance to switch plate from center
			 */
			if(this.isSwitchLeft())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 36));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 84.75));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, WALL_DISTANCE - 36));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));
			}
			else
			{
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 84.75));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, WALL_DISTANCE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));
			}
		}
		else
		{
			Logger.warning("No Position for Auto: Crossing Baseline");
			this.addSequential(new TankDriveStraightDistance(d, 0.5, 0, BASELINE_DISTANCE));
		}
	}

}
