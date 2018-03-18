package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.swerve.DriveGoToAngle;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraight;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToScaleLow;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.intake.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraightSwerve;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Auto routine to place a cube on the scale for the specified position
 */
public class AutoScale extends AutonomousRoutine
{	
	// Left or Right Scale when Position = Scale Side
	private static final double MOVE_STRAIGHT_SCALE_DISTANCE = 300;
	//private static final double MOVE_STRAIGHT_SCALE_DISTANCE_PART2 = 18;
	private static final double MOVE_TO_SCALE_DISTANCE = 10;
	// Left or Right Scale when Position != Scale Side
	private static final double MOVE_STRAIGHT_HALF_SCALE_DISTANCE = 220;
	private static final double ALIGN_TO_SCALE_DISTANCE = 225;
	private static final double AJUST_TO_SCALE_DISTANCE = 90;

	/**
	 * Auto routine to place a cube on the scale for the
	 * specified position
	 * @param position the starting position
	 * @param d the drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoScale(StartPosition position, SwerveDrive d, Elevator e, Intake i) 
	{
		super(position);
		
		try
		{
			if(position == StartPosition.LEFT)
			{
				// Starting turned 90 deg clockwise
				d.setHeadingOffset(90);

				if(isScaleLeft())
				{
					this.addSequential(new DriveStraightSwerve(d, -1, 0, MOVE_STRAIGHT_SCALE_DISTANCE));
					this.addParallel(new ElevateToScaleHigh(e));
					this.addSequential(new DriveStraight(d, -0.5, MOVE_TO_SCALE_DISTANCE));
					this.addSequential(new ReleaseCube(i));
					// TODO: Get another cube and place it on switch or scale
				}
				else
				{
					this.addSequential(new DriveStraightSwerve(d, -1, 0, MOVE_STRAIGHT_HALF_SCALE_DISTANCE));
					this.addSequential(new DriveStraight(d, -0.5, ALIGN_TO_SCALE_DISTANCE));
					this.addSequential(new DriveGoToAngle(d, 0));
					this.addParallel(new ElevateToScaleHigh(e));
					this.addSequential(new DriveStraight(d, -0.5, AJUST_TO_SCALE_DISTANCE));
					this.addSequential(new ReleaseCube(i));
					// TODO: Get another cube and place it on switch or scale
				}
			}
			else if(position == StartPosition.RIGHT)
			{
				// Starting turned 90 deg counterclockwise
				d.setHeadingOffset(-90);

				if(isScaleRight())
				{
					this.addParallel(new ElevateToScaleHigh(e));
					this.addSequential(new DriveStraightSwerve(d, 1, 0, MOVE_STRAIGHT_SCALE_DISTANCE));
					this.addSequential(new DriveStraight(d, -0.5, MOVE_TO_SCALE_DISTANCE));
					this.addSequential(new ReleaseCube(i));
					// TODO: Get another cube and place it on switch or scale
				}
				else
				{
					this.addSequential(new DriveStraightSwerve(d, 1, 0, MOVE_STRAIGHT_HALF_SCALE_DISTANCE));
					this.addSequential(new DriveStraight(d, -0.5, ALIGN_TO_SCALE_DISTANCE));
					this.addSequential(new DriveGoToAngle(d, 0));
					this.addParallel(new ElevateToScaleHigh(e));
					this.addSequential(new DriveStraight(d, -0.5, AJUST_TO_SCALE_DISTANCE));
					this.addSequential(new ReleaseCube(i));
					// TODO: Get another cube and place it on switch or scale
				}
			}
			else
			{
				String mssg;
				if(position == StartPosition.CENTER)
					mssg = "Center position not allowed for AutoScale! Crossing Auto Line...";
				else
					mssg = "No position provided for AutoScale! Crossing Auto Line...";

				Logger.warning(mssg);
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
