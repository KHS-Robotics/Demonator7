package org.usfirst.frc.team4342.robot.auton;

import org.usfirst.frc.team4342.robot.Constants;
import org.usfirst.frc.team4342.robot.OI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Superclass for Autonomous Routines
 */
public abstract class AutonomousRoutine extends CommandGroup {
	protected static final double ROBOT_X = Constants.ROBOT_WIDTH / 2;
	protected static final double ROBOT_Y = Constants.ROBOT_LENGTH / 2;
	
	protected final StartPosition position;

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
		final OI oi = OI.getInstance();
		oi.Drive.resetNavX();
		oi.Drive.setFieldOriented(false);
	}
	
	/**
	 * Gets if our switch is left
	 * @return true if left, false otherwise
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	protected static boolean isSwitchLeft() throws InvalidGameMessageException {
		return getPlateLocations().charAt(0) == 'L';
	}
	
	/**
	 * Gets if our switch is right
	 * @return true if right, false otherwise
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	protected static boolean isSwitchRight() throws InvalidGameMessageException {
		return !isSwitchLeft();
	}
	
	/**
	 * Gets if the scale is left
	 * @return true if left, false otherwise
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	protected static boolean isScaleLeft() throws InvalidGameMessageException {
		return getPlateLocations().charAt(1) == 'L';
	}
	
	/**
	 * Gets if the scale is right
	 * @return true if right, false otherwise
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	protected static boolean isScaleRight() throws InvalidGameMessageException {
		return !isScaleLeft();
	}
	
	/**
	 * Gets if both the our switch and the scale are left
	 * @return true if both are left, false otherwise
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	protected static boolean isBothLeft() throws InvalidGameMessageException {
		return isSwitchLeft() && isScaleLeft();
	}
	
	/**
	 * Gets if both the our switch and the scale are right
	 * @return true if both are right, false otherwise
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	protected static boolean isBothRight() throws InvalidGameMessageException {
		return isSwitchRight() && isScaleRight();
	}
	
	/**
	 * Gets if their switch is left
	 * @return true if left, false otherwise
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	protected static boolean isOpponentSwitchLeft() throws InvalidGameMessageException {
		return getPlateLocations().charAt(2) == 'L';
	}
	
	/**
	 * Gets if their switch is right
	 * @return true if right, false otherwise
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	protected static boolean isOpponentSwitchRight() throws InvalidGameMessageException {
		return !isOpponentSwitchLeft();
	}
	
	/**
	 * Returns three characters (R or L) specifying which plate is ours,
	 * starting with the closest plate.
	 * @return a three character string indicating which plates are ours
	 * @throws InvalidGameMessageException if the game message is malformed
	 */
	private static String getPlateLocations() throws InvalidGameMessageException {
		final String mssg = DriverStation.getInstance().getGameSpecificMessage();
		if(mssg == null || mssg.length() != 3) {
			throw new InvalidGameMessageException("Invalid game message: " + mssg);
		}
		for(char c : mssg.toCharArray()) {
			c = Character.toUpperCase(c);
			if(c != 'L' || c != 'R') {
				throw new InvalidGameMessageException("Invalid game message: " + mssg);
			}
		}

		return mssg;
	}

	/**
	 * Exception for an invalid game message
	 */
	@SuppressWarnings("serial")
	protected static class InvalidGameMessageException extends Exception {
		public InvalidGameMessageException(String message) {
			super(message);
		}
	}
}
