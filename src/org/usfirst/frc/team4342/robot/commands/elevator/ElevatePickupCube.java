package org.usfirst.frc.team4342.robot.commands.elevator;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator to pick up a cube
 */
public class ElevatePickupCube extends Elevate {
	/**
	 * Sets the height of the elevator to pick up a cube
	 * @param elevator the elevator
	 */
	public ElevatePickupCube(Elevator elevator) {
		super(elevator, 0);
	}

	/**
	 * Stops the elevator
	 */
	protected void end() {
		elevator.stop();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected boolean isFinished() {
		return super.isFinished() || elevator.isAtBottom();
	}
}
