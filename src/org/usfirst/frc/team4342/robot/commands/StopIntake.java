package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopIntake extends InstantCommand 
{

	private Intake intake;
	
	public StopIntake(Intake intake)
	{
		this.requires(intake);
		this.intake = intake;
	}
	
	@Override
	public void initialize()
	{
		intake.disabled();
	}
}
