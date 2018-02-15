package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.drive.DriveStraight;
import org.usfirst.frc.team4342.robot.commands.drive.DriveTurn;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevatePickupCube;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.intake.IntakeCube;
import org.usfirst.frc.team4342.robot.commands.intake.ReleaseCube;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Auto routine to place a cube on the switch and scale for the specified position
 * 
 * TODO: When Switch/Scale positions are opposite of our position
 * TODO: When Switch/Scale are on opposite sides
 */
public class AutoBoth extends AutonomousRoutine 
{
	// Switch'n'Scale = Position
	private static final double START_MOVE_FORWARD = 168.0;
	private static final double MOVE_TO_SWTCH = 60 + (ROBOT_X / 2);
	private static final double MOVE_TO_CUBE_PART_1 = 37 + (ROBOT_X/2);
	private static final double MOVE_TO_CUBE_PART_2 = 66.5 - (ROBOT_X/2);
	private static final double MOVE_TO_SCALE_PART_1 = 66.5 + (ROBOT_X/2);
	private static final double MOVE_TO_SCALE_PART_2 = 119 + (ROBOT_X/2);
	private static final double MOVE_TO_SCALE_PART_3 = 42 - (ROBOT_X/2);

	/**
	 * Auto routine to place a cube on the switch and scale for the
	 * specified position
	 * @param position the starting position
	 * @param d the drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoBoth(StartPosition position, SwerveDrive d, Elevator e, Intake i) {
		
		super(position);

		try
		{
			// Switch and Scale plates are on same side as start position
			if((position == StartPosition.LEFT && isBothLeft()) || (position == StartPosition.RIGHT && isBothRight())) 
			{
				final boolean turnRight = isBothLeft();

				this.addParallel(new ElevateToSwitch(e));
				this.addSequential(new DriveStraight(d, 0.5, START_MOVE_FORWARD));
				this.addSequential(new DriveTurn(d, turnRight));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_SWTCH));
				this.addSequential(new ReleaseCube(i));	
				this.addParallel(new ElevatePickupCube(e));
				this.addSequential(new DriveStraight(d, -0.5, MOVE_TO_SWTCH));
				this.addSequential(new DriveTurn(d, !turnRight));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_CUBE_PART_1));
				this.addSequential(new DriveTurn(d, turnRight));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_CUBE_PART_2));
				this.addSequential(new DriveTurn(d, turnRight));
				this.addSequential(new IntakeCube(i));
				this.addParallel(new ElevateToScaleNeutral(e));
				this.addSequential(new DriveTurn(d, turnRight));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_SCALE_PART_1));
				this.addSequential(new DriveTurn(d, turnRight));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_SCALE_PART_2));
				this.addSequential(new DriveTurn(d, turnRight));
				this.addSequential(new DriveStraight(d, 0.5, MOVE_TO_SCALE_PART_3));
				this.addSequential(new ReleaseCube(i));
			}
			else
			{
				Logger.warning("No position provided for AutoBoth! Crossing Baseline...");
				this.addSequential(new AutoBaseline(position, d));
			}
		}
		catch(InvalidGameMessageException ex)
		{
			Logger.error("Invalid game message!", ex);
		}
	}
}
