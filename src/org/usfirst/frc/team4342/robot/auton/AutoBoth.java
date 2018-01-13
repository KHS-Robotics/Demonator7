package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.IntakeCube;
import org.usfirst.frc.team4342.robot.commands.PlaceCube;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraight;
import org.usfirst.frc.team4342.robot.commands.TankGoToAngle;
import org.usfirst.frc.team4342.robot.logging.Logger;

public class AutoBoth extends AutonomousRoutine {
	
	public AutoBoth(StartPosition position) {
		
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(this.isBothLeft())
			{
				this.addSequential(new TankDriveStraight(SCALE_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
				this.addSequential(new TankDriveStraight(-SCALE_SIDE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new IntakeCube());
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new TankDriveStraight(-(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(-8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
			}
			else if(this.isSwitchLeft())
			{
				this.addSequential(new TankDriveStraight(SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch());
				this.addSequential(new PlaceCube());	
				this.addSequential(new TankDriveStraight(-SCALE_SIDE));
			}
			else
			{
				
			}
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isBothRight())
			{
				this.addSequential(new TankDriveStraight(SCALE_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
				this.addSequential(new TankDriveStraight(-SCALE_SIDE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE - SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new IntakeCube());
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE - 261.47));
				this.addSequential(new TankDriveStraight(-(PAST_SWITCH_DISTANCE - 261.47)));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(-8)); //distance to first cube  ¯\_(ツ)_/¯
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
				
				
			}
			else if(this.isSwitchRight())
			{
				this.addSequential(new TankDriveStraight(SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(SWITCH_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
				this.addSequential(new TankDriveStraight(-SWITCH_SIDE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				
				
			}
			else
			{
				
			}
		}
		else
		{
			Logger.warning("No Position for Auto: Crossing Baseline");
			this.addSequential(new TankDriveStraight(BASELINE_DISTANCE));
		}
	}

}
