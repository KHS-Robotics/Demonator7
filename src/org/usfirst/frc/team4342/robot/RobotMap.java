package org.usfirst.frc.team4342.robot;

import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// NavX Board
	public static final Port NAVX_PORT = Port.kMXP;
	public static final byte NAVX_UPDATE_RATE_HZ = (byte) 50;
	
	// Xbox Controller and Switch Box
	public static final int XBOX_PORT = 0;
	public static final int SWITCH_BOX = 1;
	
	// Motors for Swerve
	public static final int FRONT_RIGHT_DRIVE = 3;
	public static final int FRONT_RIGHT_PIVOT = 13;
	public static final int FRONT_LEFT_DRIVE = 11; 
	public static final int FRONT_LEFT_PIVOT = 0;
	public static final int REAR_RIGHT_DRIVE = 12;
	public static final int REAR_RIGHT_PIVOT = 15;
	public static final int REAR_LEFT_DRIVE = 14;
	public static final int REAR_LEFT_PIVOT = 4; 

	// Motors for Intake, Climber and Elevator
	public static final int INTAKE_MOTOR = 8;
	public static final int CLIMBER_MOTOR = 9;
	public static final int ELE_MOTOR = 10;
	
	// Drive/Translational Encoders for Swerve
	public static final int FRONT_RIGHT_DRIVE_ENC_A = 0;
	public static final int FRONT_RIGHT_DRIVE_ENC_B = 1;
	public static final int FRONT_LEFT_DRIVE_ENC_A = 2;
	public static final int FRONT_LEFT_DRIVE_ENC_B = 3;
	public static final int REAR_RIGHT_DRIVE_ENC_A = 4;
	public static final int REAR_RIGHT_DRIVE_ENC_B = 5;
	public static final int REAR_LEFT_DRIVE_ENC_A = 6;
	public static final int REAR_LEFT_DRIVE_ENC_B = 7;
	// Pivot/Rotational Analog Inputs for Swerve
	public static final int FRONT_RIGHT_PIVOT_CHANNEL = 0;
	public static final int FRONT_LEFT_PIVOT_CHANNEL = 1;
	public static final int REAR_RIGHT_PIVOT_CHANNEL = 2;
	public static final int REAR_LEFT_PIVOT_CHANNEL = 3;

	// Encoder and Limit Switch for Elevator
	public static final int ELE_ENC_IN = 8;
	public static final int ELE_ENC_OUT = 9;
	public static final int ELE_LS = 10;

	// RGB LEDs on PCM
	public static final int LED_POWER = 0;
	public static final int RED_LED = 1;
	public static final int GREEN_LED = 2;
	public static final int BLUE_LED = 3;
}
