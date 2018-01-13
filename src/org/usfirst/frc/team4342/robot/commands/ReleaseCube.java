package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Intake;

public class ReleaseCube extends CommandBase {
	
	private Intake intake;
	
	public ReleaseCube(Intake intake)
	{
		super(2);
		
		this.intake = intake;
		this.requires(intake);
	}

	@Override
	protected void initialize() {
		intake.release();
	}

	@Override
	protected void execute() {}

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	@Override
	protected void end() {
		intake.disable();
	}

}