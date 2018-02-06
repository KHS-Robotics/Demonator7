package org.usfirst.frc.team4342.robot.commands;

/**
 * Default commands for teleop. For example the drive or elevator
 */
public abstract class TeleopCommand extends CommandBase {
    /**
     * Returns false
     * @return false
     */
    @Override
    protected boolean isFinished() {
        return false;
    }
}
