package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraightDistance;
import org.usfirst.frc.team4342.robot.commands.TurnTank;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

public class AutoScale extends AutonomousRoutine{
	
	public AutoScale(StartPosition position, TankDrive d, Elevator e, Intake i) {
		super(position);
		
		if(position == StartPosition.LEFT)
		{
			if(this.isScaleLeft())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SCALE_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, PAST_SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 264));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
			}
		}
		else if(position == StartPosition.RIGHT)
		{
			if(this.isScaleRight())
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SCALE_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
				this.addSequential(new ReleaseCube(i));	
			}
			else
			{
				this.addSequential(new TankDriveStraightDistance(d, 0.5, PAST_SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d, false));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, 264));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SCALE_DISTANCE - PAST_SWITCH_DISTANCE));
				this.addSequential(new TurnTank(d));
				this.addSequential(new TankDriveStraightDistance(d, 0.5, SCALE_SIDE));
				this.addSequential(new ElevateToScaleNeutral(e));
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
