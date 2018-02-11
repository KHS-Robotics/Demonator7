package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Drive Swerve With Two Joysticks
 */
public class DriveSwerveWithTwoJoysticks extends TeleopCommand {
	private static final double DEADBAND = 0.02;

	private boolean idle;

    private Joystick xy;
    private Joystick z;
	private SwerveDrive drive;
	
	/**
	 * Drive Swerve With Two Joysticks
	 * @param xy the joystick to control the forward/backward and strafing
     * @param z the joystick to control rotation
	 * @param drive the swerve drive
	 */
	public DriveSwerveWithTwoJoysticks(Joystick xy, Joystick z, SwerveDrive drive) {
        this.xy = xy;
        this.z = z;
		this.drive = drive;
		
		this.requires(drive.fr);
		this.requires(drive.fl);
		this.requires(drive.rr);
		this.requires(drive.rl);
		
		this.requires(drive);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void initialize() {
		drive.stop();
	}

    /**
     * {@inheritDoc}
     */
	@Override
	protected void execute() {
		double xInput = xy.getX();
		double yInput = xy.getY();
		double zInput = z.getTwist();

		boolean x = Math.abs(xInput) > DEADBAND;
		boolean y = Math.abs(yInput) > DEADBAND;
		boolean Z = Math.abs(zInput) > DEADBAND;

		xInput = x ? xInput : 0;
		yInput = y ? yInput : 0;
		zInput = Z ? zInput : 0;

		if(x || y || Z) {
			drive.set(xInput, yInput, zInput);
			idle = false;
		}
		else if(!idle) {
			drive.stop();
			idle = true;
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void end() {
		drive.stop();
	}
}
