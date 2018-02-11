package org.usfirst.frc.team4342.robot.commands.tuning;

import org.usfirst.frc.team4342.robot.commands.TeleopCommand;

public abstract class PIDTuner extends TeleopCommand {
    protected abstract void setPID();
    protected abstract void setSetpoint();
    protected abstract void resetPIDValues();

    @Override
    protected void initialize() {
        
    }

    @Override
    protected void execute() {
        setPID();
        setSetpoint();
    }

    @Override
    protected void end() {
        resetPIDValues();
    }
}
