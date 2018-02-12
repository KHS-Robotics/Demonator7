package org.usfirst.frc.team4342.robot.commands.elevator;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator to place a cube
 * on the switch
 */
public class ElevateToSwitch extends Elevate {
	private static final double SWITCH_HEIGHT = 15; // TODO: Get height in inches

	/**
	 * Sets the height of the elevator to place a cube
	 * on the switch
	 * @param elevator the elevator
	 */
	public ElevateToSwitch(Elevator elevator) {
		super(elevator, SWITCH_HEIGHT);
	}
}