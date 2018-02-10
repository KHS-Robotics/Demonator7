package org.usfirst.frc.team4342.robot.tuning;

import org.usfirst.frc.team4342.robot.Constants;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to tune elevator PID
 */
public class ElevatorPIDTuner extends Thread implements Runnable {
    private Elevator elevator;

    public ElevatorPIDTuner(Elevator elevator) {
        this.elevator = elevator;

        SmartDashboard.putNumber("Elevator-P", SmartDashboard.getNumber("Elevator-P", 0.0));
        SmartDashboard.putNumber("Elevator-I", SmartDashboard.getNumber("Elevator-I", 0.0));
        SmartDashboard.putNumber("Elevator-D", SmartDashboard.getNumber("Elevator-D", 0.0));
        SmartDashboard.putNumber("Elevator-Setpoint", SmartDashboard.getNumber("Elevator-Setpoint", 0.0));
    }
    
    @Override
    public void run() {
        while(!Thread.interrupted()) {
            final double position = elevator.getPosition();
            
            if(RobotState.isTest()) {
                elevator.setPID(
                    SmartDashboard.getNumber("Elevator-P", 0.0),
                    SmartDashboard.getNumber("Elevator-I", 0.0),
                    SmartDashboard.getNumber("Elevator-D", 0.0)
                );

                elevator.setSetpoint(SmartDashboard.getNumber("Elevator-Setpoint", position));
            }
            
            SmartDashboard.putNumber("Elevator-Distance", position);

            try {
                Thread.sleep(20);
            } catch(InterruptedException ex) {
                elevator.setPID(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D);
            }
        }

        elevator.setPID(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D);
    }
}
