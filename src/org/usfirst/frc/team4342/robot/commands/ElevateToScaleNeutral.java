package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator to place a cube on the scale when
 * no one has ownership
 */
public class ElevateToScaleNeutral extends Elevate {
	/**
	 * Sets the height of the elevator to place a cube on the scale when
	 * no one has ownership
	 * @param elevator the elevator
	 */
	public ElevateToScaleNeutral(Elevator elevator) {
		super(elevator, 66);
	}
}
