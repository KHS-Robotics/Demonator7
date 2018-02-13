package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.commands.TeleopCommand;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Drive Swerve With Xbox Controller
 */
public class DriveSwerveWithXbox extends TeleopCommand {
    private static final double DEADBAND = 0.02;

    private boolean idle;
    
    private XboxController controller;
    private SwerveDrive drive;

    /**
     * Drive Swerve With Xbox Controller
     * @param controller the xbox one controller
     * @param drive the swerve drive
     */
    public DriveSwerveWithXbox(XboxController controller, SwerveDrive drive) {
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
		boolean y = Math.abs(yInput) > DEADBAND;
        boolean z = Math.abs(zInput) > DEADBAND;
        
        xInput = x ? xInput : 0;
		yInput = y ? yInput : 0;
		zInput = z ? zInput : 0;

        if(x || y || z) {
            drive.set(xInput, -yInput, zInput);
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
