package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.TankDriveStraight;

public class AutoBaseline extends AutonomousRoutine {
	
	public AutoBaseline(StartPosition position) {
		super(position);
		
		this.addSequential(new TankDriveStraight(BASELINE_DISTANCE));
	}

}
