package org.usfirst.frc.team4342.robot.commands;

import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

public class TurnTank extends TankGoToAngle {
    private boolean clockwise;

    public TurnTank(TankDrive drive, boolean clockwise) {
        super(drive, 0);

        this.clockwise = clockwise;
    }

    public TurnTank(TankDrive drive) {
        this(drive, true);
    }

    @Override
    protected void initialize() {
        if(clockwise) {
            this.setSetpoint(drive.getHeading() + 90);
        } else {
            this.setSetpoint(drive.getHeading() - 90);
        }
        
        super.initialize();
    }
}