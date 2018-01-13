package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.TankDriveStraightDistance;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

public class AutoBaseline extends AutonomousRoutine {
	
	public AutoBaseline(StartPosition position, TankDrive drive) {
		super(position);
		
		this.addSequential(new TankDriveStraightDistance(drive, 0.5, 0, BASELINE_DISTANCE));
	}

}
