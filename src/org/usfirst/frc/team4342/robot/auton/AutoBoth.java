package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.DriveStraight;
import org.usfirst.frc.team4342.robot.commands.DriveTurn;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.IntakeCube;
import org.usfirst.frc.team4342.robot.commands.ReleaseCube;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.DriveTrainBase;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;

/**
 * Auto routine to place a cube on the switch and scale for the
 * specified position
 * 
 * TODO: Check math for distances and angles
 */
public class AutoBoth extends AutonomousRoutine 
{
	/**
	 * Auto routine to place a cube on the switch and scale for the
	 * specified position
	 * @param position the starting position
	 * @param d the drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoBoth(StartPosition position, DriveTrainBase d, Elevator e, Intake i) {
		
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(this.isBothLeft())
			{
				this.addSequential(new DriveStraight(d, 0.5, SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				this.addSequential(new DriveStraight(d, 0.5, -SCALE_SIDE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, 8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new IntakeCube(i));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new DriveStraight(d, 0.5, -(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, -8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
			}
			else if(this.isSwitchLeft())
			{
				this.addSequential(new DriveStraight(d, 0.5, SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));	
				this.addSequential(new DriveStraight(d, 0.5, -SCALE_SIDE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, 8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new DriveTurn(d));
				this.addSequential(new IntakeCube(i));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new DriveStraight(d, 0.5, -(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, 264 - 8));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				
			}
			else
			{
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, 185.5)); //distance from expected position to other plate
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - 261.47)); //261.47 is far wall of switch
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));
			}
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isBothRight())
			{
				this.addSequential(new DriveStraight(d, 0.5, SCALE_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				this.addSequential(new DriveStraight(d, 0.5, -SCALE_SIDE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, 8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new IntakeCube(i));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new DriveStraight(d, 0.5, -(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, -8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				
				
			}
			else if(this.isSwitchRight())
			{
				this.addSequential(new DriveStraight(d, 0.5, SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));	
				this.addSequential(new DriveStraight(d, 0.5, -SCALE_SIDE));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, 8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new IntakeCube(i));
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new DriveStraight(d, 0.5, -(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new DriveTurn(d));
				this.addSequential(new DriveStraight(d, 0.5, 264 - 8));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new DriveStraight(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				
				
			}
			else
			{
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE));
				this.addSequential(new DriveTurn(d, false));
				this.addSequential(new DriveStraight(d, 0.5, 185.5)); //distance from expected position to other plate
				this.addSequential(new DriveStraight(d, 0.5, PAST_SWITCH_DISTANCE - 261.47)); //261.47 is far wall of switch
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));
			}
		}
		else
		{
			Logger.warning("No Position for Both Auto! Crossing Baseline...");
			this.addSequential(new AutoBaseline(position, d));
		}
	}
}
