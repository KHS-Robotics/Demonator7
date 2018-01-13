package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

public class Elevate extends CommandBase {
	
	private Elevator elevator;
	private int height;

	public Elevate(Elevator elevator, int height)
	{
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
		return elevator.isAtSetpoint();
	}

	@Override
	protected void end() {
		
	}

}
