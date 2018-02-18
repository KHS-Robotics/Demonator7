package org.usfirst.frc.team4342.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Base class for a drive train
 */
public abstract class DriveTrainBase extends SubsystemBase implements PIDSource, PIDOutput {
    private double p, i, d;
    private final PIDController yawPID;

    private double offset;
    private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;

    private final AHRS navx;
    
    /**
     * Drive Train base class
     * @param navx the NavX
     */
    public DriveTrainBase(AHRS navx) {
        this.navx = navx;

        yawPID = new PIDController(p, i, d, this, this);
        yawPID.setInputRange(-180.0, 180.0);
		yawPID.setOutputRange(-1.0, 1.0);
		yawPID.setContinuous();
		yawPID.setAbsoluteTolerance(4);
    }

    /**
     * Go Straight
     * @param direction the direction ranging from -1 to 1
     * @param yaw the heading to maintain from -180.0 to 180.0
     */
    public abstract void goStraight(double direction, double yaw);

    /**
     * Gets the remaining distance based on the average of all encoder distances
     * @param distance the desired distance
     * @param distances the distances of each encoder in a respective 
     * order decided by the implementation of {@link #getAllDistances()}
     * @return the remaining distance
     */
    public abstract double remainingDistance(double distance, double[] distances);

    /**
     * Gets all distances of each encoder on the drive train. This method
     * intended to used by {@link #remainingDistance(double, double[])}.
     * @return the distances of each encoder on the drive train
     */
    public abstract double[] getAllDistances();
    
    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
    	disablePID();
    }

    /**
     * Sets the PID values for the internal PID controller for yaw.
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivatve value
     */
    public void setPID(double p, double i, double d) {
        this.p = p;
		this.i = i;
		this.d = d;
        yawPID.setPID(p, i, d);
    }

     /**
     * Sets the P value for the internal PID controller for yaw.
     * @param p the proportional value
     */
    public void setP(double p) {
        this.p = p;
        setPID(p, i, d);
    }

    /**
     * Gets the proportional value for the internal PID controller for yaw.
     * @return the proportional value
     */
    public double getP() {
        return p;
    }

    /**
     * Sets the I value for the internal PID controller for yaw.
     * @param i the integral value
     */
    public void setI(double i) {
        this.i = i;
        setPID(p, i, d);
    }

    /**
     * Gets the integral value for the internal PID controller for yaw.
     * @return the integral value
     */
    public double getI() {
        return i;
    }

    /**
     * Sets the D value for the intenral PID controller for yaw.
     * @param d the derivative value
     */
    public void setD(double d) {
        this.d = d;
        setPID(p, i, d);
    }

    /**
     * Gets the derivative value for the internal PID controller for yaw.
     * @return the derivative value
     */
    public double getD() {
        return d;
    }

    /**
     * Resets the NavX
     */
    public void resetNavX() {
        navx.reset();
    }

    /**
	 * Sets the heading offset to return the proper values from the NavX
	 * @param offset the offset ranging from -180.0 to 180.0 degrees
	 */
	public void setHeadingOffset(double offset) {
		this.offset = offset;
	}

    /**
	 * Sets the internal PID conroller's setpoint to the specified yaw and enables PID
	 * @param yaw the yaw to orient the robot to
	 */
	public void setHeading(double yaw) {
		yawPID.setSetpoint(normalizeYaw(yaw));
		enablePID();
    }
    
    /**
	 * Enables the internal PID controller
	 */
	public void enablePID() {
		yawPID.enable();
	}
	
	/**
	 * Disables the internal PID controller if it's enabled
	 */
	public void disablePID() {
		if(yawPID.isEnabled())
			yawPID.disable();
	}

    /**
	 * Gets if the internal PID controller is on target with its setpoint
	 * within a tolerance of two degrees
	 * @return true if the internal PID controller is at its setpoint, false otherwise
	 */
	public boolean onTarget() {
		return yawPID.onTarget();
	}

    /**
	 * Gets the current heading of the robot
	 * @return the current heading of the robot ranging from -180.0 to 180.0 degrees
	 */
	public double getHeading() {
		return normalizeYaw(navx.getYaw() + getOffset());
    }

    /**
     * Gets the heading offset
     * @return the heading offset
     */
    public double getOffset() {
        return offset;
    }
    
    /**
     * Gets the current angle of the robot
     * @return the current angle of the robot
     */
    public double getAngle() {
        return navx.getAngle();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double pidGet() {
        if(pidSourceType == PIDSourceType.kRate)
			return navx.getRate();
		else
			return this.getHeading();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSourceType = pidSource;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSourceType;
    }

    /**
	 * Internal function to normalize yaw
	 * @param yaw the unnormalized yaw
	 * @return the normalized yaw ranging from -180.0 to 180.0
	 */
	private static double normalizeYaw(double yaw) {
		while(yaw >= 180)
			yaw -= 360;
		while(yaw <= -180)
			yaw += 360;
		
		return yaw;
	}
}
