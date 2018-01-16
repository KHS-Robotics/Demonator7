package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Drive Swerve With Two Joysticks
 */
public class DriveSwerveWithTwoJoysticks extends CommandBase {
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
		drive.set(xy.getX(), xy.getY(), z.getTwist());
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void end() {
		drive.stop();
	}
    
    /**
     * {@inheritDoc}
     */
	@Override
	protected boolean isFinished() {
		return false;
	}
}
