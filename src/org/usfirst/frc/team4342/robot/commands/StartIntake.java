package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StartIntake extends InstantCommand 
{

	private Intake intake;
	
	public StartIntake(Intake intake)
	{
		this.requires(intake);
		this.intake = intake;
	}
	
	@Override
	protected void initialize()
	{
		intake.enable();
	}

}
