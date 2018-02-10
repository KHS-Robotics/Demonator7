package org.usfirst.frc.team4342.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4342.robot.auton.AutoBaseline;
import org.usfirst.frc.team4342.robot.auton.AutoBoth;
import org.usfirst.frc.team4342.robot.auton.AutoScale;
import org.usfirst.frc.team4342.robot.auton.AutoSwitch;
import org.usfirst.frc.team4342.robot.auton.AutonomousRoutine;
import org.usfirst.frc.team4342.robot.auton.Priority;
import org.usfirst.frc.team4342.robot.auton.StartPosition;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.logging.PDPLogger;
import org.usfirst.frc.team4342.robot.tuning.DrivePIDTuner;
import org.usfirst.frc.team4342.robot.tuning.ElevatorPIDTuner;
import org.usfirst.frc.team4342.robot.tuning.PivotPIDTuner;

/**
 * Main Robot Class
 * @author FRC Team 4342
 */
public class Robot extends TimedRobot {
	private SendableChooser<StartPosition> startPositionChooser;
	private SendableChooser<Priority> priorityChooser;
	private AutonomousRoutine autonomousRoutine;

	// For test mode
	private DrivePIDTuner driveTuner;
	private ElevatorPIDTuner elevTuner;
	private PivotPIDTuner frTuner, flTuner, rrTuner, rlTuner;
	
	/**
	 * Robot-wide initialization code
	 */
	@Override
	public void robotInit() {
		Logger.info("Bootstrapping Demonator7...");
		
		final OI oi = OI.getInstance();
		PDPLogger.start();
		AwesomeLights.start();
		
		Logger.info("Initializing autonomous choosers...");
		startPositionChooser = new SendableChooser<StartPosition>();
		startPositionChooser.addDefault("Left", StartPosition.LEFT);
		startPositionChooser.addObject("Center", StartPosition.CENTER);
		startPositionChooser.addObject("Right", StartPosition.RIGHT);
		SmartDashboard.putData("Start Position Chooser", startPositionChooser);
		
		priorityChooser = new SendableChooser<Priority>();
		priorityChooser.addDefault("Baseline", Priority.BASELINE);
		priorityChooser.addObject("Switch", Priority.SWITCH);
		priorityChooser.addObject("Scale", Priority.SCALE);
		priorityChooser.addObject("Both", Priority.BOTH);
		SmartDashboard.putData("Priority Chooser", priorityChooser);


		Logger.info("Linking subsystems to SmartDashboard...");
		// Power Distribution Panel
		SmartDashboard.putData("PDP", oi.PDP);
		// Scheduler
		SmartDashboard.putData("Scheduler", Scheduler.getInstance());
		// Subsystems
		SmartDashboard.putData("Drive", oi.Drive);
		SmartDashboard.putData("Elevator", oi.Elevator);
		SmartDashboard.putData("Intake", oi.Intake);
		SmartDashboard.putData("Climber", oi.Climber);
		
		Logger.info("Finished bootstrapping Demonator7.");
	}
	
	/**
	 * Initialization code for teleop (operator control) mode
	 */
	@Override
	public void teleopInit() {
		Logger.info("Entering teleop...");
		stopAutonomousRoutine();
	}

	/**
	 * Periodic code for teleop (operator control) mode
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
	
	/**
	 * Initialization code for autonomous mode 
	 */
	@Override
	public void autonomousInit() {
		Logger.info("Entering autonomous...");
		stopAutonomousRoutine();
		setAutonomousRoutine();
		startAutonomousRoutine();
	}
	
	/**
	 * Periodic code for autonomous mode
	 */
	@Override
	public void autonomousPeriodic() {
		if(autonomousRoutine != null)
			Scheduler.getInstance().run();
	}
	
	/**
	 * Initialization code for disabled mode
	 */
	@Override
	public void disabledInit() {
		Logger.info("Entering disabled...");
		stopAutonomousRoutine();
		stopPIDTuning();
	}
	
	/**
	 * Initialization code for test mode 
	 */
	@Override
	public void testInit() {
		Logger.info("Entering test...");
		startPIDTuning();
	}
	
	/**
	 * Starts the autonomous routine
	 */
	private void startAutonomousRoutine() {
		if(autonomousRoutine != null && !autonomousRoutine.isRunning()) {
			Logger.info("Starting autonomous routine...");
			autonomousRoutine.start();
		}
	}
	
	/**
	 * Stops the autonomous routine
	 */
	private void stopAutonomousRoutine() {
		if(autonomousRoutine != null) {
			Logger.info("Stopping autonomous routine...");
			autonomousRoutine.cancel();
			Scheduler.getInstance().run();
		}
	}

	/**
	 * Sets the autonomous routine based on the Sendable Choosers
	 */
	private void setAutonomousRoutine() {
		final OI oi = OI.getInstance();

		String routine;
		final StartPosition position = startPositionChooser.getSelected();
		switch(priorityChooser.getSelected()) {
			case BASELINE:
				routine = "AutoBaseline";
				autonomousRoutine = new AutoBaseline(position, oi.Drive);
			break;
			
			case SWITCH:
				routine = "AutoSwitch";
				autonomousRoutine = new AutoSwitch(position, oi.Drive, oi.Elevator, oi.Intake);
			break;
			
			case SCALE:
				routine = "AutoScale";
				autonomousRoutine = new AutoScale(position, oi.Drive, oi.Elevator, oi.Intake);
			break;
				
			case BOTH:
				routine = "AutoBoth";
				autonomousRoutine = new AutoBoth(position, oi.Drive, oi.Elevator, oi.Intake);
			break;
			
			default:
				routine = null;
				Logger.warning("Auto priority could not be determined! Crossing auto line!");
				autonomousRoutine = new AutoBaseline(position, oi.Drive);
		}

		if(routine != null)
			Logger.info("Selected \"" + routine + "\" routine for " + position.toString().toLowerCase());
	}

	/**
	 * Starts PID tuning
	 */
	private void startPIDTuning() {
		Logger.info("Starting PID tuning...");

		LiveWindow.setEnabled(false);
		final OI oi = OI.getInstance();

		// Drive
		driveTuner = new DrivePIDTuner(oi.Drive);
		// Elevator
		elevTuner = new ElevatorPIDTuner(oi.Elevator);
		// Swerve Modules
		frTuner = new PivotPIDTuner(oi.FR, "FR", 
			Constants.Drive.PivotPID.FR_P,
			Constants.Drive.PivotPID.FR_I,
			Constants.Drive.PivotPID.FR_D
		);
		flTuner = new PivotPIDTuner(oi.FL, "FL", 
			Constants.Drive.PivotPID.FL_P,
			Constants.Drive.PivotPID.FL_I,
			Constants.Drive.PivotPID.FL_D
		);
		rrTuner = new PivotPIDTuner(oi.RR, "RR", 
			Constants.Drive.PivotPID.RR_P,
			Constants.Drive.PivotPID.RR_I,
			Constants.Drive.PivotPID.RR_D
		);
		rlTuner = new PivotPIDTuner(oi.RL, "RL", 
			Constants.Drive.PivotPID.RL_P,
			Constants.Drive.PivotPID.RL_I,
			Constants.Drive.PivotPID.RL_D
		);

		driveTuner.start();
		elevTuner.start();
		frTuner.start();
		flTuner.start();
		rrTuner.start();
		rlTuner.start();
	}

	/**
	 * Stops PID tuning
	 */
	private void stopPIDTuning() {
		Logger.info("Stopping PID tuning...");
		
		if(driveTuner != null)
			driveTuner.interrupt();
		if(elevTuner != null)
			elevTuner.interrupt();
		if(frTuner != null)
			frTuner.interrupt();
		if(flTuner != null)
			flTuner.interrupt();
		if(rrTuner != null)
			rrTuner.interrupt();
		if(rlTuner != null)
			rlTuner.interrupt();

		driveTuner = null;
		elevTuner = null;
		frTuner = null;
		flTuner = null;
		rrTuner = null;
		rlTuner = null;
	}
}
