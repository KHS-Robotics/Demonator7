package org.usfirst.frc.team4342.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command to wait
 */
public class Wait extends Command {
    /**
     * Command to wait
     * @param duration the length of time to wait
     */
    public Wait(double duration) {
        super(duration);
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
}
