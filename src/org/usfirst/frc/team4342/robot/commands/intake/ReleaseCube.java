package org.usfirst.frc.team4342.robot.commands.intake;

import org.usfirst.frc.team4342.robot.commands.CommandBase;
import org.usfirst.frc.team4342.robot.subsystems.Intake;

/**
 * Command to release a cube in autonomous. This command
 * will enable the release for 2 seconds.
 */
public class ReleaseCube extends CommandBase
{
	private Intake intake;
	
	/**
	 * Command to release a cube in autonomous. This command
	 * will enable the release for 2 seconds.
	 * @param intake the intake
	 */
	public ReleaseCube(Intake intake)
	{
		super(1.5);
		
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
		intake.stop();
	}
}
