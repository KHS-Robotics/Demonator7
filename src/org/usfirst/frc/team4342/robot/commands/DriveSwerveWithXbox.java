package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Drive Swerve With Xbox Controller
 */
public class DriveSwerveWithXbox extends CommandBase {
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
        drive.set(
            controller.getX(Hand.kLeft), 
            controller.getY(Hand.kLeft), 
            controller.getX(Hand.kRight)
        );
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
