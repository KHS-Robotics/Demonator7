package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraightDistance;
import org.usfirst.frc.team4342.robot.commands.TurnTank;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

/**
 * Auto routine to place a cube on the scale for the
 * specified position
 */
public class AutoScale extends AutonomousRoutine
{	
	/**
	 * Auto routine to place a cube on the scale for the
	 * specified position
	 * @param position the starting position
	 * @param d the tank drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoScale(StartPosition position, TankDrive d, Elevator e, Intake i) 
	{
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(this.isScaleLeft())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, MOVE_STRAIGHT_SCALE_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, MOVE_TO_SCALE_DISTANCE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, MOVE_STRAIGHT_HALF_SCALE_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, ALIGN_TO_SCALE_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, AJUST_TO_SCALE_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
			}
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isScaleRight())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, MOVE_STRAIGHT_SCALE_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, MOVE_TO_SCALE_DISTANCE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, MOVE_STRAIGHT_HALF_SCALE_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, ALIGN_TO_SCALE_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, AJUST_TO_SCALE_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));
			}
		}
		else
		{
			Logger.warning("No Position for Auto: Crossing Baseline");
			this.addSequential(new AutoBaseline(position, d));
		}
	}

}
