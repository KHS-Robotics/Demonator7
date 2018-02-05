package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.OI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Superclass for Autonomous Routines
 */
public abstract class AutonomousRoutine extends CommandGroup {
	protected final StartPosition position;

	// TODO: Update robot dimensions
	protected static final double ROBOT_X = 32.5 / 2;
	protected static final double ROBOT_Y = 27.5 / 2;

	/**
	 * Constructs an autonomous routine
	 * @param position the starting position of the robot
	 * @param priority
	 */
	public AutonomousRoutine(StartPosition position) {
		this.position = position;
	}
	
	@Override
	protected void initialize() {
		OI.getInstance().Drive.resetNavX();
	}
	
	/**
	 * Gets if our switch is left
	 * @return true if left, false otherwise
	 */
	protected static boolean isSwitchLeft() {
		return getPlateLocations().charAt(0) == 'L';
	}
	
	/**
	 * Gets if our switch is right
	 * @return true if right, false otherwise
	 */
	protected static boolean isSwitchRight() {
		return !isSwitchLeft();
	}
	
	/**
	 * Gets if the scale is left
	 * @return true if left, false otherwise
	 */
	protected static boolean isScaleLeft() {
		return getPlateLocations().charAt(1) == 'L';
	}
	
	/**
	 * Gets if the scale is right
	 * @return true if right, false otherwise
	 */
	protected static boolean isScaleRight() {
		return !isScaleLeft();
	}
	
	/**
	 * Gets if both the our switch and the scale are left
	 * @return true if both are left, false otherwise
	 */
	protected static boolean isBothLeft() {
		return isSwitchLeft() && isScaleLeft();
	}
	
	/**
	 * Gets if both the our switch and the scale are right
	 * @return true if both are right, false otherwise
	 */
	protected static boolean isBothRight() {
		return isSwitchRight() && isScaleRight();
	}
	
	/**
	 * Gets if their switch is left
	 * @return true if left, false otherwise
	 */
	protected static boolean isOpponentSwitchLeft() {
		return getPlateLocations().charAt(2) == 'L';
	}
	
	/**
	 * Gets if their switch is right
	 * @return true if right, false otherwise
	 */
	protected static boolean isOpponentSwitchRight() {
		return !isOpponentSwitchLeft();
	}
	
	/**
	 * Returns three characters (R or L) specifying which plate is ours,
	 * starting with the closest plate.
	 * @return a three character string indicating which plates are ours
	 */
	private static String getPlateLocations() {
		return DriverStation.getInstance().getGameSpecificMessage();
	}
}
