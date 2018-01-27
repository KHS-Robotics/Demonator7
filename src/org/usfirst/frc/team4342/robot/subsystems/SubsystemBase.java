package org.usfirst.frc.team4342.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Superclass of all subsystems
 */
public abstract class SubsystemBase extends Subsystem {
	/**
	 * Sets the default command to <code>null</code>
	 */
	@Override
	protected void initDefaultCommand() {
		this.setDefaultCommand(null);
	}
}
