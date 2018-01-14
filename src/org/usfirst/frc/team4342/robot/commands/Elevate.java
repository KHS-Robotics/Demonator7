package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

public class Elevate extends CommandBase {
	private Elevator elevator;
	private double height;

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
