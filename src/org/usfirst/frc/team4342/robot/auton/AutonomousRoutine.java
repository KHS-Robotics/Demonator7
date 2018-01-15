package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.OI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Superclass for Autonomous Routines
 */
public abstract class AutonomousRoutine extends CommandGroup {
	protected final StartPosition position;
	
	/**
	 * have to update robot dimensions
	 */
	protected double current = OI.getInstance().TankDrive.getHeading();
	
	protected final double ROBOT_X = 23.5 / 2;
	protected final double ROBOT_Y = 32.3 / 2;
	protected final double BASELINE_DISTANCE = 120 - ROBOT_Y;
	protected final double WALL_DISTANCE = 140 - ROBOT_Y;
	protected final double SWITCH_DISTANCE = 168 - ROBOT_Y;
	protected final double SWITCH_SIDE = 37.5 - ROBOT_Y; //check math
	protected final double SCALE_SIDE = 23.57 - ROBOT_Y; //check math
	protected final double PAST_SWITCH_DISTANCE = 281.47 - ROBOT_Y;
	protected final double SCALE_DISTANCE = 324 - ROBOT_Y;
	protected final double LEFT_TURN = 90 + current;
	protected final double RIGHT_TURN = -90 + current;
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
	 * @return true if right, false otherwise
	 */
	protected final boolean isOpponentSwitchRight() {
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
