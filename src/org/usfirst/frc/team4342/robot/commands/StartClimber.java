package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Starts the climber motor
 */
public class StartClimber extends InstantCommand 
{
	private Climber climber;
	
	/**
	 * Starts the climber motor
	 * @param climber the climber
	 */
	public StartClimber(Climber climber)
	{
		this.requires(climber);
		this.climber = climber;
	}
	
	@Override
	protected void initialize() 
	{
		climber.enable();
	}
}
