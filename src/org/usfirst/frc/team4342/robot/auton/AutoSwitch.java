package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.PlaceCube;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraight;
import org.usfirst.frc.team4342.robot.commands.TankGoToAngle;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;

public class AutoSwitch extends AutonomousRoutine {
	
	public AutoSwitch(StartPosition position, Elevator e) {
		
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(this.isSwitchLeft())
			{
				this.addSequential(new TankDriveStraight(SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new PlaceCube());	
			}
			else
			{
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(185.5)); //distance from expected position to other plate
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE - 261.47)); //261.47 is far wall of switch
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new PlaceCube());
			}
				
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isSwitchRight())
			{
				this.addSequential(new TankDriveStraight(SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(SWITCH_SIDE));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new PlaceCube());
			}
			else
			{
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(185.5)); //distance from expected position to other plate
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(PAST_SWITCH_DISTANCE - 261.47)); //261.47 is far wall of switch
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new PlaceCube());
			}
		}
		else if(position == StartPosition.CENTER)
		{
			/**
			 * 36 is to avoid the exchange, 84.75 distance to switch plate from center
			 */
			if(this.isSwitchLeft())
			{
				this.addSequential(new TankDriveStraight(36));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(84.75));
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(WALL_DISTANCE - 36));
				this.addSequential(new ElevateToSwitch(e));
				this.addSequential(new PlaceCube());
			}
			else
			{
				this.addSequential(new TankGoToAngle(RIGHT_TURN));
				this.addSequential(new TankDriveStraight(84.75));
				this.addSequential(new TankGoToAngle(LEFT_TURN));
				this.addSequential(new TankDriveStraight(WALL_DISTANCE));
				this.addSequential(new ElevateToSwitch(e));
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
