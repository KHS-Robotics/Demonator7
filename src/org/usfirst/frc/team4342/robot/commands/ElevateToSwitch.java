package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator to place a cube
 * on the switch
 */
public class ElevateToSwitch extends Elevate {
	/**
	 * Sets the height of the elevator to place a cube
	 * on the switch
	 * @param elevator the elevator
	 */
	public ElevateToSwitch(Elevator elevator) {
		super(elevator, 15);
	}
}
