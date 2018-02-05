package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Intake;

/**
 * Command to intake a cube in autonomous. This command
 * will enable the intake for 2 seconds.
 */
public class IntakeCube extends CommandBase 
{
	private Intake intake;
	
	/**
	 * Command to intake a cube in autonomous. This command
	 * will enable the intake for 2 seconds.
	 * @param intake the intake
	 */
	public IntakeCube(Intake intake)
	{
		super(2);
		
		this.intake = intake;
		this.requires(intake);
	}

	@Override
	protected void initialize() {
		intake.enable();
	}

	@Override
	protected void execute() {}

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	@Override
	protected void end() {
		intake.stop();
	}
}
