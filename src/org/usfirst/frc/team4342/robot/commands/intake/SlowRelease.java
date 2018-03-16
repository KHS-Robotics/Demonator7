package org.usfirst.frc.team4342.robot.commands.intake;

import org.usfirst.frc.team4342.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Starts the intake to pick up a cube
 */
public class SlowRelease extends InstantCommand 
{
	private Intake intake;
	
	/**
	 * Starts the intake
	 * @param intake the intake
	 */
	public SlowRelease(Intake intake)
	{
		this.requires(intake);
		this.intake = intake;
	}
	
	@Override
	protected void initialize()
	{
		intake.slow();
	}

}
