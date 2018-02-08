
package org.usfirst.frc.team4342.robot;

/**
 * Class to keep track of the button maps for teleop
 */
public class ButtonMap 
{
	/**
	 * Xbox Controller to control the Swerve Drive
	 */
	public static class DriveController
	{
		public static final int GO_STRAIGHT = XboxButton.kA.value;
		// TODO: Implement Directional Pad (D-Pad) values in Button enum
		public static final int GO_TO_ZERO = -1; // TODO: up on D-Pad
		public static final int GO_TO_LEFT = -1; // TODO: left on D-Pad
		public static final int GO_TO_RIGHT = -1; // TODO: right on D-Pad
		public static final int GO_TO_180 = -1; // TODO: down on D-Pad
	}
	
	/**
	 * Switch Box to control the Accumualtor. Note that
	 * switches 3, 7, 8, 9, and 12 are reversed. Meaning,
	 * they read true when flipped down and false when flipped up.
	 */
	public static class SwitchBox
	{
		public static final int INTAKE = 5;
		public static final int RELEASE = 4;
		public static final int CLIMB = 7;
		public static final int RESET = 10;
		public static final int ELEVATE_SCALE_HIGH = 2;
		public static final int ELEVATE_SCLALE_NEUTRAL = 11;
		public static final int ELEVATE_SCALE_LOW = 1;
		public static final int ELEVATE_SWITCH = 12;
		public static final int ELEVATE_PICKUP_CUBE = 3;
		public static final int ELEVATOR_OVERIDE = 9;
	}

	/**
	 * Represents a digital button on an XboxController.
	 * // TODO: Implement Directional Pad (D-Pad) values
	 */
	 enum XboxButton {
		kBumperLeft(5),
		kBumperRight(6),
		kStickLeft(9),
		kStickRight(10),
		kA(1),
		kB(2),
		kX(3),
		kY(4),
		kBack(7),
		kStart(8);

		private int value;

		XboxButton(int value) {
			this.value = value;
		}
	}
}
