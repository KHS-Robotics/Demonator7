package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

public class Elevate extends CommandBase {
	
	private Elevator elevator;
	private int height;

	public Elevate(Elevator elevator, int height)
	{
		this.elevator = elevator;
		this.height = height;
	}
	@Override
	protected void initialize() {
		
	}

	@Override
	protected void execute() {
		elevator.setSetpoint(height);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return elevator.isAtSetpoint();
	}

	@Override
	protected void end() {
		
	}

}
