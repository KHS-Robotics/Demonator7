package org.usfirst.frc.team4342.robot.commands.elevator;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator to place a cube on the scale when
 * we have ownership
 */
public class ElevateToScaleLow extends Elevate {
	private static final double SCALE_HEIGHT = 2000;

	/**
	 * Sets the height of the elevator to place a cube on the scale when
	 * we have ownership
	 * @param elevator the elevator
	 */
	public ElevateToScaleLow(Elevator elevator) {
		super(elevator, SCALE_HEIGHT);
	}
}
