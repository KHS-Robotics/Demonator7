package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.commands.CrossBaseline;
import org.usfirst.frc.team4342.robot.logging.Logger;

public class AutoSwitch extends AutonomousRoutine {
	
	public AutoSwitch(StartPosition position, Priority priority) {
		
		super(position, priority);
		
		if(position == StartPosition.LEFT)
		{
			
		}
		else if(position == StartPosition.RIGHT)
		{
			
		}
		else if(position == StartPosition.CENTER)
		{
			
		}
		else
		{
			Logger.warning("No Position for Auto: Crossing Baseline");
			this.addSequential(new CrossBaseline());
		}
	}

}
