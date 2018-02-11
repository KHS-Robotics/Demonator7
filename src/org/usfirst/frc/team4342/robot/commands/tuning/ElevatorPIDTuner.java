package org.usfirst.frc.team4342.robot.commands.tuning;

import org.usfirst.frc.team4342.robot.Constants;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to tune elevator PID
 */
public class ElevatorPIDTuner extends PIDTuner {
    private Elevator elevator;

    public ElevatorPIDTuner(Elevator elevator) {
        this.elevator = elevator;
        
        this.requires(elevator);

        SmartDashboard.putNumber("Elevator-P", SmartDashboard.getNumber("Elevator-P", 0.0));
        SmartDashboard.putNumber("Elevator-I", SmartDashboard.getNumber("Elevator-I", 0.0));
        SmartDashboard.putNumber("Elevator-D", SmartDashboard.getNumber("Elevator-D", 0.0));
        SmartDashboard.putNumber("Elevator-Setpoint", SmartDashboard.getNumber("Elevator-Setpoint", 0.0));
    }

    @Override
    protected void setPID() {
        elevator.setPID(
            SmartDashboard.getNumber("Elevator-P", 0.0),
            SmartDashboard.getNumber("Elevator-I", 0.0),
            SmartDashboard.getNumber("Elevator-D", 0.0)
        );
    }

    @Override
    protected void setSetpoint() {
        final double position = elevator.getPosition();
        SmartDashboard.putNumber("Elevator-Distance", position);
        elevator.setSetpoint(SmartDashboard.getNumber("Elevator-Setpoint", position));
    }

    @Override
    protected void resetPIDValues() {
        elevator.setPID(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D);
    }
}
