package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator to place a cube on the scale when
 * the opponent has ownership
 */
public class ElevateToScaleHigh extends Elevate {
	private static final double SCALE_HEIGHT = 76; // TODO: Get height in inches

	/**
	 * Sets the height of the elevator to place a cube on the scale when
	 * the opponent has ownership
	 * @param elevator the elevator
	 */
	public ElevateToScaleHigh(Elevator elevator) {
		super(elevator, SCALE_HEIGHT);
	}
}
