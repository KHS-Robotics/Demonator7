package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.PlaceCube;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraight;
import org.usfirst.frc.team4342.robot.commands.TankGoToAngle;
import org.usfirst.frc.team4342.robot.logging.Logger;

public class AutoScale extends AutonomousRoutine{
	
	public AutoScale(StartPosition position) {
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(this.isScaleLeft())
			{
				this.addSequential(new TankDriveStraight(SCALE_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
			}
			else
			{
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(264));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
			}
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isScaleRight())
			{
				this.addSequential(new TankDriveStraight(SCALE_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
			}
			else
			{
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(264));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral());
				this.addSequential(new PlaceCube());	
			}
		}
		else
		{
			Logger.warning("No Position for Auto: Crossing Baseline");
			this.addSequential(new TankDriveStraight(BASELINE_DISTANCE));
		}
	}

}
