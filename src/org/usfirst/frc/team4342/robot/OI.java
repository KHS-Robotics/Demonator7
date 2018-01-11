package org.usfirst.frc.team4342.robot;

import com.kauailabs.navx.frc.AHRS;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public final AHRS NavX;
	
	private OI() {
		NavX = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);
	}

	private static OI instance;
	
	public static OI getInstance() {
		if(instance == null)
			instance = new OI();
		
		return instance;
	}
}
