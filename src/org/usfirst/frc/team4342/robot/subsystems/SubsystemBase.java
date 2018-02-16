package org.usfirst.frc.team4342.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Superclass of all subsystems
 */
public abstract class SubsystemBase extends Subsystem {
	/**
     * Stops all motors associated with the subsystem
     */
	public abstract void stop();

	/**
	 * Sets the default command to <code>null</code>
	 */
	@Override
	protected void initDefaultCommand() {
		this.setDefaultCommand(null);
	}

	/**
	 * <p>Builds values to send to the <code>SmartDashboard</code></p>
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		
	}
}
