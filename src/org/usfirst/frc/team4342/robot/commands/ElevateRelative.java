package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator relative to its current setpoint
 */
public class ElevateRelative extends Elevate {
	private double delta;

    /**
	 * Sets the height of the elevator relative to its current setpoint
	 * @param elevator the elevator
	 * @param delta the desired change in setpoint
	 */
	public ElevateRelative(Elevator elevator, double delta) {
		super(elevator, 0.0);

		this.delta = delta;
	}

	@Override
	protected void initialize() {
		elevator.setSetpointRelative(delta);
	}

	@Override
	protected boolean isFinished() {
		return super.isFinished() || elevator.isAtBottom();
	}
}
