package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Command to control the elevator with a joystick
 */
public class ElevateWithJoystick extends CommandBase {
	private static final double DEADBAND = 0.03, TOP_WINDOW = 70.0, BOTTOM_WINDOW = 5.0;
	
	private boolean idle, initializedIdle;
	
	private Joystick joystick;
	private Elevator elevator;
	
	/**
	 * Command to control the elevator with a joystick
	 * @param joystick the joystick
	 * @param elevator the elevator
	 */
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
		final double INPUT = joystick.getTwist();
		idle = checkJoystickDeadband(INPUT);
		
		if(elevator.isAtBottom()) {
			elevator.stop();
			elevator.reset();
			
			if(!idle && INPUT < 0) {
				return;
			}
		}
		
		if(!elevator.isAtBottom() && idle && !initializedIdle) {
			elevator.setSetpoint(elevator.getDistance()); // hold current height
			initializedIdle = true;
		} 
		else if(!idle) {
			final boolean IN_BOTTOM_WINDOW = elevator.getDistance() < BOTTOM_WINDOW;
			final boolean IN_TOP_WINDOW = elevator.getDistance() > TOP_WINDOW;

			double input = 0.0;

			if(IN_TOP_WINDOW && INPUT > 0)
				input = 0.12;
			else if(IN_BOTTOM_WINDOW && INPUT < 0)
				input = -0.06;
			else
				input = INPUT;

			elevator.set(input);
			initializedIdle = false;
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		elevator.stop();
	}
	
	private static boolean checkJoystickDeadband(double a) {
		return Math.abs(a) < DEADBAND;
	}
}
