package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Joystick;

public class ElevateWithJoystick extends CommandBase {
	private static final double DEADBAND = 0.03;
	
	private boolean idle;
	private double idleSetpoint;
	
	private Joystick joystick;
	private Elevator elevator;
	
	public ElevateWithJoystick(Joystick joystick, Elevator elevator) {
		this.joystick = joystick;
		this.elevator = elevator;
		
		this.requires(elevator);
	}

	@Override
	protected void initialize() {
		
	}

	@Override
	protected void execute() {
		final double OUTPUT = -joystick.getY();
		final boolean OUTPUT_ABOVE_DEADBAND = checkJoystickDeadband(OUTPUT);
		final boolean IN_BOTTOM_WINDOW = elevator.getDistance() < 5;
		final boolean IN_TOP_WINDOW = elevator.getDistance() > 70;
		
		if(elevator.isAtBottom()) {
			elevator.stop();
			elevator.reset();
			
			if(OUTPUT_ABOVE_DEADBAND && OUTPUT < 0) {
				return;
			}
		}
		
		if(!elevator.isAtBottom() && !OUTPUT_ABOVE_DEADBAND && !idle) {
			idleSetpoint = elevator.getDistance();
			elevator.setSetpoint(idleSetpoint);
			idle = true;
			
			return;
		}
		
		if(IN_TOP_WINDOW && OUTPUT_ABOVE_DEADBAND && OUTPUT > 0) {
			elevator.set(0.1);
			idle = false;
			
			return;
		}
		else if(IN_BOTTOM_WINDOW && OUTPUT_ABOVE_DEADBAND && OUTPUT < 0) {
			elevator.set(-0.06);
			idle = false;
			
			return;
		}
		
		if(OUTPUT_ABOVE_DEADBAND) {
			elevator.set(OUTPUT);
			idle = false;
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		elevator.set(0);
	}
	
	private static boolean checkJoystickDeadband(double a) {
		return Math.abs(a) > DEADBAND;
	}
}
