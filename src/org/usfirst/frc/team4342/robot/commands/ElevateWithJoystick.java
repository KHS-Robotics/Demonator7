package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Command to control the elevator with a joystick
 */
public class ElevateWithJoystick extends CommandBase {
	private static final double DEADBAND = 0.03, TOP_WINDOW = 70.0, BOTTOM_WINDOW = 5.0;
	
	private boolean idle, initializedIdle;
	
	private Joystick joystick;
	private JoystickButton override;
	private Elevator elevator;
	
	/**
	 * Command to control the elevator with a joystick
	 * @param joystick the joystick
	 * @param elevator the elevator
	 */
	public ElevateWithJoystick(Joystick joystick, JoystickButton override, Elevator elevator) {
		this.joystick = joystick;
		this.override = override;
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

		// Emergency override in case sensors malfunction
		if(!idle && override.get()) {
			elevator.disable();
			elevator.set(INPUT);
			return;
		}
		
		if(elevator.isAtBottom()) {
			elevator.stop();
			elevator.reset();
			
			if(!idle && INPUT < 0) {
				return;
			}
		}
		
		if(!elevator.isAtBottom() && idle && !initializedIdle) {
			elevator.setSetpoint(elevator.getPosition()); // hold current height
			elevator.enable();
			initializedIdle = true;
		} 
		else if(!idle) {
			final boolean IN_BOTTOM_WINDOW = elevator.getPosition() < BOTTOM_WINDOW;
			final boolean IN_TOP_WINDOW = elevator.getPosition() > TOP_WINDOW;

			double input = 0.0;

			if(IN_TOP_WINDOW && INPUT > 0)
				input = 0.12;
			else if(IN_BOTTOM_WINDOW && INPUT < 0)
				input = -0.06;
			else
				input = INPUT;

			elevator.disable();
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
