package org.usfirst.frc.team4342.robot.auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Superclass for Auto Routines
 */
public abstract class AutonomousRoutine extends CommandGroup {
	private final StartPosition position;
	private final Priority priority;
	
	/**
	 * Constructs an autonomous routine
	 * @param position the starting position of the robot
	 * @param priority
	 */
	public AutonomousRoutine(StartPosition position, Priority priority) {
		this.position = position;
		this.priority = priority;
	}
	
	/**
	 * Gets the routine's starting position
	 * @return the routine's starting position
	 * @see StartPosition
	 */
	protected final StartPosition getPosition() {
		return position;
	}
	
	/**
	 * Gets the priority for the routine
	 * @return the priority for the routine
	 * @see Priority
	 */
	protected final Priority getPriority() {
		return priority;
	}
	
	/**
	 * Gets if our switch is left
	 * @return true if left, false otherwise
	 */
	protected final boolean isSwitchLeft() {
		return getPlateLocations().charAt(0) == 'L';
	}
	
	/**
	 * Gets if our switch is right
	 * @return true if right, false otherwise
	 */
	protected final boolean isSwitchRight() {
		return !isSwitchLeft();
	}
	
	/**
	 * Gets if the scale is left
	 * @return true if left, false otherwise
	 */
	protected final boolean isScaleLeft() {
		return getPlateLocations().charAt(1) == 'L';
	}
	
	/**
	 * Gets if the scale is right
	 * @return true if right, false otherwise
	 */
	protected final boolean isScaleRight() {
		return !isScaleLeft();
	}
	
	/**
	 * Gets if both the our switch and the scale are left
	 * @return true if both are left, false otherwise
	 */
	protected final boolean isBothLeft() {
		return isSwitchLeft() && isScaleLeft();
	}
	
	/**
	 * Gets if both the our switch and the scale are right
	 * @return true if both are right, false otherwise
	 */
	protected final boolean isBothRight() {
		return isSwitchRight() && isScaleRight();
	}
	
	/**
	 * Gets if their switch is left
	 * @return true if left, false otherwise
	 */
	protected final boolean isOpponentSwitchLeft() {
		return getPlateLocations().charAt(2) == 'L';
	}
	
	/**
	 * Gets if their switch is right
	 * @return true if left, false otherwise
	 */
	protected final boolean isOpponentSwitchRight() {
		return !isOpponentSwitchLeft();
	}
	
	/**
	 * Returns three characters (R or L) specifying which plate is ours,
	 * starting with the closest plate.
	 * @return three characters (R or L)
	 */
	private static String getPlateLocations() {
		return DriverStation.getInstance().getGameSpecificMessage();
	}
}
