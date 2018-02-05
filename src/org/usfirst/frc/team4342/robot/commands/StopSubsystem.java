package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SubsystemBase;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Stops a subsystem
 */
public class StopSubsystem extends InstantCommand {
    private SubsystemBase subsystem;

    /**
     * Stops a subsystem
     * @param subsystem the subsystem
     */
    public StopSubsystem(SubsystemBase subsystem) {
        this.subsystem = subsystem;

        this.requires(subsystem);
    }

    @Override
    protected void initialize() {
        subsystem.stop();
    }
}
