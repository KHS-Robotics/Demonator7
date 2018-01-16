package org.usfirst.frc.team4342.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Superclass of all commands
 */
public abstract class CommandBase extends Command {
	/**
	 * {@inheritDoc}
	 */
	public CommandBase() {
		super();
	}
	
	/**
	 * {@inheritDoc}
	 */
	public CommandBase(double timeout) {
		super(timeout);
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
