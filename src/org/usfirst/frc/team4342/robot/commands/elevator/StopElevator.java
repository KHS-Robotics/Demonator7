package org.usfirst.frc.team4342.robot.commands.elevator;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Stops the elevator
 */
public class StopElevator extends InstantCommand {
    private Elevator elevator;

    /**
     * Stops the elevator
     * @param elevator the elevator
     */
    public StopElevator(Elevator elevator) {
        this.elevator = elevator;
        this.requires(elevator);
    }

    @Override
    protected void initialize() {
        elevator.stop();
    }
}
