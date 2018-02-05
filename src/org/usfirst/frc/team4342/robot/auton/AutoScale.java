package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.DriveStraight;
import org.usfirst.frc.team4342.robot.commands.DriveTurn;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ReleaseCube;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;

/**
 * Auto routine to place a cube on the scale for the
 * specified position
 */
public class AutoScale extends AutonomousRoutine
{	
	// Left or Right Scale when Position = Scale Side
	private static final double MOVE_STRAIGHT_SCALE_DISTANCE = 324;
	private static final double MOVE_TO_SCALE_DISTANCE = 42 - ROBOT_X;
	// Left or Right Scale when Position != Scale Side
	private static final double MOVE_STRAIGHT_HALF_SCALE_DISTANCE = 210;
	private static final double ALIGN_TO_SCALE_DISTANCE = 222;
	private static final double AJUST_TO_SCALE_DISTANCE = 114;

	/**
	 * Auto routine to place a cube on the scale for the
	 * specified position
	 * @param position the starting position
	 * @param d the drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoScale(StartPosition position, DriveTrainBase d, Elevator e, Intake i) 
	{
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(isScaleLeft())
			{
				this.addParallel(new ElevateToScaleNeutral(e));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_STRAIGHT_SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_SCALE_DISTANCE));
				this.addSequential(new ReleaseCube(i));	
			}
			else
			{
				this.addSequential(new DriveStraight(d, 0.5, MOVE_STRAIGHT_HALF_SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addParallel(new ElevateToScaleNeutral(e));
				this.addSequential(new DriveStraight(d, 0.5, ALIGN_TO_SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, AJUST_TO_SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new ReleaseCube(i));	
			}
		}
		else if(position == StartPosition.RIGHT)
		{
			if(isScaleRight())
			{
				this.addParallel(new ElevateToScaleNeutral(e));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_STRAIGHT_SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_SCALE_DISTANCE));
				this.addSequential(new ReleaseCube(i));	
			}
			else
			{
				this.addSequential(new DriveStraight(d, 0.5, MOVE_STRAIGHT_HALF_SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addParallel(new ElevateToScaleNeutral(e));
				this.addSequential(new DriveStraight(d, 0.5, ALIGN_TO_SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, AJUST_TO_SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new ReleaseCube(i));
			}
		}
		else
		{
			String mssg;
			if(position == StartPosition.CENTER)
				mssg = "Center position not allowed for AutoScale! Crossing Auto Line...";
			else
				mssg = "Center position not allowed for AutoScale! Crossing Auto Line...";

			Logger.warning(mssg);
			this.addSequential(new AutoBaseline(position, d));
		}
	}
}
