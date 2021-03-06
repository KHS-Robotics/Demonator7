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
		public static final int GO_STRAIGHT = XboxButton.kBumperRight.value;
		public static final int GO_TO_ZERO = XboxButton.kY.value;
		public static final int GO_TO_LEFT = XboxButton.kX.value;
		public static final int GO_TO_RIGHT = XboxButton.kB.value;
		public static final int GO_TO_180 = XboxButton.kA.value;
		public static final int ROBOT_ORIENT = XboxButton.kBumperLeft.value;
	}
	
	/**
	 * Switch Box to control the Accumualtor. Note that
	 * switches 3, 7, 8, 9, and 12 are reversed. Meaning,
	 * they read true when flipped down and false when flipped up.
	 */
	public static class SwitchBox
	{
		public static final int INTAKE = 1;
		public static final int RELEASE = 2;
		public static final int SLOW_RELEASE = 3;
		public static final int CLIMB = 10;
		public static final int BACKUP_CLIMB = 7;
		public static final int RESET = 8;
		//public static final int ELEVATE_SCALE_HIGH = 5;
		//public static final int ELEVATE_SCLALE_NEUTRAL = 11;
		//public static final int ELEVATE_SCALE_LOW = 4;
		//public static final int ELEVATE_SWITCH = 12;
		public static final int ELEVATE_PICKUP_CUBE = 12;
		public static final int ELEVATOR_OVERIDE = 9;
		public static final int TUNE_PID = 8;
		public static final int DRIVE_SLOW_OVERRIDE = 6;
	}

	/**
	 * Represents a digital button on an XboxController
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
