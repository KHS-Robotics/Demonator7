package org.usfirst.frc.team4342.robot;

/**
 * Class to keep track of the button maps for teleop
 */
public class ButtonMap 
{

	/**
	 * Drive Sticks to control the Tank Drive
	 */
	public static class DriveStick
	{
		/**
		 * Right Drive Stick
		 */
		public static class Right
		{
			public static final int GO_STRAIGHT = 1;
			public static final int GO_TO_ZERO = 3;
			public static final int GO_TO_LEFT = 4;
			public static final int GO_TO_RIGHT = 5;
			public static final int GO_180 = 2;
		}
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
		public static final int OVERIDE = 9;
	}
}
