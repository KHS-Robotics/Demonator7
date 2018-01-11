package org.usfirst.frc.team4342.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command.  You can replace me with your own command.
 */
public abstract class CommandBase extends Command {
	public CommandBase() {
		// use this.requires(subsystem) here
	}
	
	/**
	 * Calls {@link #end()} if {@link #cancel()} is called.
	 */
	@Override
	protected void interrupted() {
		this.end();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected abstract void initialize();

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected abstract void execute();

	
	/**
	 * {@inheritDoc}
	 */
	@Override
	protected abstract boolean isFinished();

	
	/**
	 * {@inheritDoc}
	 */
	@Override
	protected abstract void end();
}
