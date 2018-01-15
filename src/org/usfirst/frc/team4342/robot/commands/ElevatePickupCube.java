package org.usfirst.frc.team4342.robot.commands;

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
}
