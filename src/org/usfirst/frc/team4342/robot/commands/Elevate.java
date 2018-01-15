package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

/**
 * Elevate Command to set the elevator to a specified height
 */
public class Elevate extends CommandBase {
	private Elevator elevator;
	private double height;

	/**
	 * Elevate Command to set the elevator to a specified height
	 * @param elevator the elevator
	 * @param height the desired height of the elevator
	 */
	public Elevate(Elevator elevator, double height) {
		super(5);
		
		this.elevator = elevator;
		this.height = height;
		
		this.requires(elevator);
	}
	
	@Override
	protected void initialize() {
		elevator.setSetpoint(height);
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return elevator.isAtSetpoint() || this.isTimedOut();
	}

	@Override
	protected void end() {
		
	}
}
