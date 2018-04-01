package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.swerve.DriveGoToAngle;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraight;
import org.usfirst.frc.team4342.robot.auton.AutonomousRoutine.InvalidGameMessageException;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevatePickupCube;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.intake.IntakeCube;
import org.usfirst.frc.team4342.robot.commands.intake.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraightSwerve;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Auto routine to place a cube on the switch for the specified position
 */
public class AutoCenter2Cube extends AutonomousRoutine 
{	
	// Center Switch
		//private static final double CENTER_DIAGONAL_DISTANCE = 88 + ROBOT_Y;
		private static final double CENTER_DIAGONAL_DISTANCE = 97 + ROBOT_Y;
		// Left or Right Switch
		// Start Position and Switch location are the same
		private static final double LEFT_RIGHT_PANEL_ALIGN_DISTANCE = 155 - ROBOT_X;
		private static final double LEFT_RIGHT_SWITCH_DISTANCE = 40 - (ROBOT_Y);
		// Start Position and Switch location are opposite
		private static final double LEFT_RIGHT_PAST_SWITCH_DISTANCE = 198;
		private static final double LEFT_RIGHT_PAST_SWITCH_ALIGN_DISTANCE = 155 - (ROBOT_X/2);
		private static final double LEFT_RIGHT_MOVE_TO_SWITCH_DISTANCE = 50;
		
		private static final double MOVE_STRAIGHT_SCALE_DISTANCE = 300;
		private static final double MOVE_TO_SCALE_DISTANCE = 10;

		private static final double CENTER_MOVE_BACKWARDS = 30;
		private static final double MOVE_TO_SECOND_CUBE = 20;
		/**
		 * Auto routine to place a cube on the switch for the
		 * specified position
		 * @param position the starting position
		 * @param d the drive
		 * @param e the elevator
		 * @param i the intake
		 * @see StartPosition
		 */
		public AutoCenter2Cube(StartPosition position, SwerveDrive d, Elevator e, Intake i) 
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
						this.addSequential(new DriveStraightSwerve(d, -0.8, 0.0, LEFT_RIGHT_PANEL_ALIGN_DISTANCE));
						this.addSequential(new DriveStraight(d, -0.5, LEFT_RIGHT_SWITCH_DISTANCE));
						this.addSequential(new ReleaseCube(i));
						this.addSequential(new DriveStraight(d, 0.6, LEFT_RIGHT_SWITCH_DISTANCE));
						// TODO: Pick up another cube and put in on our switch plate or scale
					}
					else
					{
//						this.addSequential(new DriveStraightSwerve(d, -0.9, 0.0, LEFT_RIGHT_PAST_SWITCH_DISTANCE));
//						this.addParallel(new ElevateToSwitch(e));
//						this.addSequential(new DriveStraight(d, -0.7, LEFT_RIGHT_PAST_SWITCH_ALIGN_DISTANCE));
//						this.addSequential(new DriveGoToAngle(d, 150));
//						this.addSequential(new DriveStraight(d, -0.5, LEFT_RIGHT_MOVE_TO_SWITCH_DISTANCE));
//						this.addSequential(new ReleaseCube(i));
						
						if(isScaleLeft())
						{
							this.addSequential(new DriveStraightSwerve(d, -1, 0, MOVE_STRAIGHT_SCALE_DISTANCE));
							this.addParallel(new ElevateToScaleHigh(e));
							this.addSequential(new DriveStraight(d, -0.5, MOVE_TO_SCALE_DISTANCE));
							this.addSequential(new ReleaseCube(i));
							this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_SCALE_DISTANCE));
						}
						else
						{
							this.addSequential(new DriveStraightSwerve(d, -0.8, 0.0, LEFT_RIGHT_PANEL_ALIGN_DISTANCE));
						}
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
						this.addSequential(new DriveStraightSwerve(d, 0.8, 0.0, LEFT_RIGHT_PANEL_ALIGN_DISTANCE));
						this.addSequential(new DriveStraight(d, -0.5, LEFT_RIGHT_SWITCH_DISTANCE));
						this.addSequential(new ReleaseCube(i));
						this.addSequential(new DriveStraight(d, 0.6, LEFT_RIGHT_SWITCH_DISTANCE));
						// TODO: Pick up another cube and put in on our switch plate or scale
					}
					else
					{
//						this.addSequential(new DriveStraightSwerve(d, 0.9, 0.0, LEFT_RIGHT_PAST_SWITCH_DISTANCE));
//						this.addSequential(new DriveStraight(d, -0.8, LEFT_RIGHT_PAST_SWITCH_ALIGN_DISTANCE));
//						this.addSequential(new DriveGoToAngle(d, -142));
//						this.addParallel(new ElevateToSwitch(e));
//						this.addSequential(new DriveStraight(d, -0.5, LEFT_RIGHT_MOVE_TO_SWITCH_DISTANCE));
//						this.addSequential(new ReleaseCube(i));
						
						if(isScaleRight())
						{
							this.addParallel(new ElevateToScaleHigh(e));
							this.addSequential(new DriveStraightSwerve(d, 1, 0, MOVE_STRAIGHT_SCALE_DISTANCE));
							this.addSequential(new DriveStraight(d, -0.5, MOVE_TO_SCALE_DISTANCE));
							this.addSequential(new ReleaseCube(i));
							this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_SCALE_DISTANCE));
						}
						else
						{
							this.addSequential(new DriveStraightSwerve(d, 0.8, 0.0, LEFT_RIGHT_PANEL_ALIGN_DISTANCE));
						}
						// TODO: Pick up another cube and put in on our switch plate or scale
					}
				}
				else if(position == StartPosition.CENTER)
				{
					final double xSpeed = isSwitchRight() ? 0.3 : -0.35;
					final double offset = isSwitchRight() ? -7 : 0;
					this.addParallel(new ElevateToSwitch(e));
					this.addSequential(new DriveStraightSwerve(d, xSpeed, -0.6, CENTER_DIAGONAL_DISTANCE + offset));
					this.addSequential(new ReleaseCube(i));
					this.addSequential(new DriveStraightSwerve(d, -xSpeed, 0.6, CENTER_MOVE_BACKWARDS + offset));
					this.addParallel(new ElevatePickupCube(e));
					this.addSequential(new DriveGoToAngle(d, (isSwitchRight() ? -60 : 60)));
					this.addSequential(new DriveStraight(d, -0.6, MOVE_TO_SECOND_CUBE));
					this.addParallel(new IntakeCube(i));
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
