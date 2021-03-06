package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.commands.TeleopCommand;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Drive Swerve Slow With Xbox Controller
 */
public class DriveSwerveSlowWithXbox extends TeleopCommand {
    // Multiplier to divide input
    private static final double INPUT_MULTIPLYER = 0.60;
    // Deadbands
    private static final double YDEADBAND = 0.08;
    private static final double DEADBAND = 0.05;

    private boolean idle;
    
    private XboxController controller;
    private SwerveDrive drive;

    /**
     * Drive Swerve Slow With Xbox Controller
     * @param controller the xbox one controller
     * @param drive the swerve drive
     */
    public DriveSwerveSlowWithXbox(XboxController controller, SwerveDrive drive) {
        this.controller = controller;
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
        double xInput = controller.getX(Hand.kLeft);
		double yInput = controller.getY(Hand.kLeft);
		double zInput = controller.getX(Hand.kRight);

        boolean x = Math.abs(xInput) > DEADBAND;
		boolean y = Math.abs(yInput) > YDEADBAND;
        boolean z = Math.abs(zInput) > DEADBAND;
        
        xInput = x ? xInput : 0;
		yInput = y ? yInput : 0;
		zInput = z ? zInput : 0;

        if(x || y || z) {
            drive.set(xInput*INPUT_MULTIPLYER, yInput*INPUT_MULTIPLYER, zInput*INPUT_MULTIPLYER);
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
