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
}
