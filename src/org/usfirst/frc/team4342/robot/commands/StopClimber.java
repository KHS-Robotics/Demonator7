package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopClimber extends InstantCommand 
{

	private Climber climber;
	
	public StopClimber(Climber climber)
	{
		this.requires(climber);
		this.climber = climber;
	}
	
	@Override
	protected void initialize() 
	{
		climber.disabled();
	}

}