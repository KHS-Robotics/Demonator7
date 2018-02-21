package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.commands.TeleopCommand;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Command used to control swerve like a tank drive. This should
 * only be used in unfortunate situations...
 */
public class DriveSwerveTankWithJoysticks extends TeleopCommand {
    private static final double DEADBAND = 0.06;

	private boolean idle;

	private XboxController xbox;
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
		
		this.requires(drive.fr);
		this.requires(drive.fl);
		this.requires(drive.rr);
		this.requires(drive.rl);
		
		this.requires(drive);
	}

	/**
	 * Drive Swerve "Tank" With Joysticks
	 * @param xbox the xbox controller
	 * @param drive the swerve drive
	 */
	public DriveSwerveTankWithJoysticks(XboxController xbox, SwerveDrive drive) {
		this.xbox = xbox;
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
        double leftInput = getLeftInput();
		double rightInput = getRightInput();

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

	private double getLeftInput() {
		return xbox != null ? -xbox.getY(Hand.kLeft) : -left.getY();
	}

	private double getRightInput() {
		return xbox != null ? -xbox.getY(Hand.kRight) : -right.getY();
	}
}
