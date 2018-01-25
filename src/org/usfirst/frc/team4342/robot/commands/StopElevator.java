package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Command to stop the elevator
 */
public class StopElevator extends InstantCommand {
    private Elevator elevator;

    /**
     * Command to stop the elevator
     * @param e the elevator
     */
    public StopElevator(Elevator e) {
        elevator = e;
    }

    @Override
    protected void initialize() {
        elevator.stop();
    }
}
