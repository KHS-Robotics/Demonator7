package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Command used to control swerve like a tank drive. This should
 * only be used in unfortunate situations...
 */
public class DriveSwerveTankWithJoysticks extends CommandBase {
    private static final double DEADBAND = 0.02;

	private boolean idle;

	private Joystick left, right;
	private SwerveDrive drive;
	
	/**
	 * Drive Swerve "Tank" With Joysticks
	 * @param left the joystick to control the left side
     * @param right the joystick to control the right side
	 * @param drive the swerve drive
	 */
	public DriveSwerveTankWithJoysticks(Joystick left, Joystick right, SwerveDrive drive) {
        this.left = left;
        this.right = right;
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
        double leftInput = -left.getY();
		double rightInput = -right.getY();

		boolean l = Math.abs(leftInput) > DEADBAND;
		boolean r = Math.abs(rightInput) > DEADBAND;

		leftInput = l ? leftInput : 0;
		rightInput = r ? rightInput : 0;

		if(l || r) {
			drive.set(leftInput, rightInput);
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

    @Override
    protected boolean isFinished() {
        return false;
    }
}
