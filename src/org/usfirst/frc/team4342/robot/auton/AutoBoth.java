package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.IntakeCube;
import org.usfirst.frc.team4342.robot.commands.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraightDistance;
import org.usfirst.frc.team4342.robot.commands.TankGoToAngle;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

/**
 * Auto routine to place a cube on the switch and scale for the
 * specified position
 */
public class AutoBoth extends AutonomousRoutine 
{
	/**
	 * Auto routine to place a cube on the switch and scale for the
	 * specified position
	 * @param position the starting position
	 * @param d the tank drive
	 * @param e the elevator
	 * @param i the intake
	 * @see StartPosition
	 */
	public AutoBoth(StartPosition position, TankDrive d, Elevator e, Intake i) {
		
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(this.isBothLeft())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 0, SCALE_DISTANCE));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -SCALE_SIDE));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, LEFT_TURN, PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, 8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new IntakeCube(i));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
			}
			else if(this.isSwitchLeft())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 0, SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));	
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -SCALE_SIDE));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, 8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new IntakeCube(i));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, 264 - 8));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 0, PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, 185.5)); //distance from expected position to other plate
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - 261.47)); //261.47 is far wall of switch
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));
			}
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isBothRight())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 0, SCALE_DISTANCE));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -SCALE_SIDE));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, 8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new IntakeCube(i));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				
				
			}
			else if(this.isSwitchRight())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 0, SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new ReleaseCube(i));	
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -SCALE_SIDE));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, 8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new IntakeCube(i));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, -(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new TankGoToAngle(d, RIGHT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, 264 - 8));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
				
				
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 0, PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(d, LEFT_TURN));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, 185.5)); //distance from expected position to other plate
				this.addSequential(new TankDriveStraightDistance(d, 0.5, current, PAST_SWITCH_DISTANCE - 261.47)); //261.47 is far wall of switch
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
