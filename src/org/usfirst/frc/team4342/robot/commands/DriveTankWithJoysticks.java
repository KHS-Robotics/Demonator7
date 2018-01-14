package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

import edu.wpi.first.wpilibj.Joystick;

public class DriveTankWithJoysticks extends CommandBase {
	private static final double JOYSTICK_DEADZONE = 0.06;
	private static final double SENSITIVITY = 0.2;
	
	private Joystick leftJoystick, rightJoystick;
	private TankDrive drive;
	
	/**
	 * Creates a new <code>DriveWithJoysticks</code> command.
	 * @param leftJoystick the left stick to control the left side of the drive train
	 * @param rightJoystick the right stick to control the right side of the drive train
	 * @param drive the <code>TankDrive</code> subsystem to output to
	 * @see org.usfirst.frc.team4342.robot.subsystems.TankDrive
	 */
	public DriveTankWithJoysticks(Joystick leftJoystick, Joystick rightJoystick, TankDrive drive)
	{
		this.requires(drive);
		
		this.leftJoystick = leftJoystick;
		this.rightJoystick = rightJoystick;
		this.drive = drive;
	}
	
	/**
	 * The main logic of this command to actually drive the robot.
	 */
	@Override
	protected void execute()
	{
		final double LEFT_Y = -leftJoystick.getY();
		final double RIGHT_Y = -rightJoystick.getY();

		if(Math.abs(LEFT_Y) > JOYSTICK_DEADZONE || Math.abs(RIGHT_Y) > JOYSTICK_DEADZONE)
		{	
			drive.set(adjust(LEFT_Y), adjust(RIGHT_Y));
		} 
		else 
		{
			drive.stop();
		}
	}
	
	/**
	 * Disables the drive PID and then zeros the outputs
	 */
	@Override
	protected void end()
	{
		drive.stop();
	}
	
	@Override
	protected boolean isFinished()
	{
		return false;
	}
	
	/**
	 * Internal function to adjust output for sensitivity control
	 * @param input the desired input before adjustment from -1.0 to 1.0
	 * @return the adjusted output
	 */
	private static double adjust(double input)
	{
		if(input > 1)
			input = 1;
		else if(input < -1)
			input = -1;
		
		return SENSITIVITY*Math.pow(input, 3) + input*(1 - SENSITIVITY);
	}
	
	/** {@inheritDoc} */
	@Override
	protected void initialize() {}
}
