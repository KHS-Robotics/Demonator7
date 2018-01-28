package org.usfirst.frc.team4342.robot;

import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static final Port NAVX_PORT = Port.kMXP;
	public static final byte NAVX_UPDATE_RATE_HZ = (byte) 50;
	
	public static final int LEFT_DRIVE_STICK = 0;
	public static final int RIGHT_DRIVE_STICK = 1;
	public static final int SWITCH_BOX = 3;
	
	public static final int FRONT_LEFT = 0;
	public static final int FRONT_RIGHT = 15;
	public static final int REAR_LEFT = 1;
	public static final int REAR_RIGHT = 14;
	public static final int INTAKE_MOTOR = 3;
	public static final int CLIMBER_MOTOR = 13;
	public static final int ELE_MOTOR = 2;
	
	public static final int LEFT_DRIVE_IN = 2;
	public static final int LEFT_DRIVE_OUT = 3;
	public static final int RIGHT_DRIVE_IN = 0;
	public static final int RIGHT_DRIVE_OUT = 1;
	public static final int ELE_ENC_IN = 4;
	public static final int ELE_ENC_OUT = 5;
	
	public static final int ELE_LS = 6;
}
