package org.usfirst.frc.team4342.robot.commands.swerve;

import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;

/**
 * Command to turn the robot a specified number of degrees
 */
public class DriveTurn extends DriveGoToAngle {
    private double offset;
    private boolean clockwise;

    /**
     * Command to turn the robot a specified number of degrees
     * @param drive the drive
     * @param offset the desired change in heading (0 to 180)
     * @param clockwise true to turn clockwise (right), false to
     * turn counterclockwise (left)
     */
    public DriveTurn(SwerveDrive drive, double offset, boolean clockwise) {
        super(drive, 0);

        this.offset = offset;
        this.clockwise = clockwise;
    }

    /**
     * Command to turn the robot 90 degrees left or right
     * @param drive the drive
     * @param clockwise true to turn clockwise (right), false to
     * turn counterclockwise (left)
     */
    public DriveTurn(SwerveDrive drive, boolean clockwise) {
        this(drive, 90, clockwise);
    }

    /**
     * Command to turn the robot 90 degrees right
     * @param drive the drive
     */
    public DriveTurn(SwerveDrive drive) {
        this(drive, 90, true);
    }

    @Override
    protected void initialize() {
        double setpoint = drive.getHeading();
        setpoint = clockwise ? setpoint + offset : setpoint - offset;
        this.yaw = setpoint;
        
        super.initialize();
    }
}