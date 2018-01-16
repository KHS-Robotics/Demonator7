package org.usfirst.frc.team4342.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4342.robot.auton.AutoBaseline;
import org.usfirst.frc.team4342.robot.auton.AutoBoth;
import org.usfirst.frc.team4342.robot.auton.AutoScale;
import org.usfirst.frc.team4342.robot.auton.AutoSwitch;
import org.usfirst.frc.team4342.robot.auton.AutonomousRoutine;
import org.usfirst.frc.team4342.robot.auton.Priority;
import org.usfirst.frc.team4342.robot.auton.StartPosition;
import org.usfirst.frc.team4342.robot.logging.DemonDashboard;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.logging.PDPLogger;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Main Robot Class
 * @author FRC Team 4342
 */
public class Robot extends TimedRobot {
	private SendableChooser<StartPosition> startPositionChooser;
	private SendableChooser<Priority> priorityChooser;
	private AutonomousRoutine autonomousRoutine;
	
	/**
	 * Robot-wide initialization code
	 */
	@Override
	public void robotInit() {
		Logger.info("Bootstrapping Demonator7...");
		
		OI.getInstance();
		DemonDashboard.start();
		PDPLogger.start();
		
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
		
		SmartDashboard.putNumber("E-P" , 0.0);
		SmartDashboard.putNumber("E-I" , 0.0);
		SmartDashboard.putNumber("E-D" , 0.0);
		
		SmartDashboard.putNumber("D-P" , 0.0);
		SmartDashboard.putNumber("D-I" , 0.0);
		SmartDashboard.putNumber("D-D" , 0.0);
		
		Logger.info("Finished bootstrapping Demonator7.");
	}
	
	/**
	 * Initialization code for teleop (operator control) mode
	 */
	@Override
	public void teleopInit() {
		stopAutonomousRoutine();
		OI.getInstance().TankDrive.setNeutralMode(NeutralMode.Brake);
		OI.getInstance().EleMotor.setNeutralMode(NeutralMode.Brake);
	}

	/**
	 * Periodic code for teleop (operator control) mode
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		double EP = SmartDashboard.getNumber("E-P", 0.0);
		double EI = SmartDashboard.getNumber("E-I", 0.0);
		double ED = SmartDashboard.getNumber("E-D", 0.0);
		OI.getInstance().Elevator.setPID(EP, EI, ED);
		
		double DP = SmartDashboard.getNumber("D-P", 0.0);
		double DI = SmartDashboard.getNumber("D-I", 0.0);
		double DD = SmartDashboard.getNumber("D-D", 0.0);
		OI.getInstance().TankDrive.setPID(DP, DI, DD);
	}
	
	/**
	 * Initialization code for autonomous mode 
	 */
	@Override
	public void autonomousInit() {
		stopAutonomousRoutine();
		
		OI.getInstance().TankDrive.setNeutralMode(NeutralMode.Brake);
		OI.getInstance().EleMotor.setNeutralMode(NeutralMode.Brake);
		
		final OI oi = OI.getInstance();

		final StartPosition position = startPositionChooser.getSelected();
		switch(priorityChooser.getSelected()) {
			case BASELINE:
				autonomousRoutine = new AutoBaseline(position, oi.TankDrive);
			break;
			
			case SWITCH:
				autonomousRoutine = new AutoSwitch(position, oi.TankDrive, oi.Elevator, oi.Intake);
			break;
			
			case SCALE:
				autonomousRoutine = new AutoScale(position, oi.TankDrive, oi.Elevator, oi.Intake);
			break;
				
			case BOTH:
				autonomousRoutine = new AutoBoth(position, oi.TankDrive, oi.Elevator, oi.Intake);
			break;
			
			default:
				Logger.warning("Auto priority could not be determined! Crossing auto line!");
				autonomousRoutine = new AutoBaseline(position, oi.TankDrive);
		}
		
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
		stopAutonomousRoutine();
		OI.getInstance().TankDrive.setNeutralMode(NeutralMode.Coast);
		OI.getInstance().EleMotor.setNeutralMode(NeutralMode.Coast);
		Scheduler.getInstance().run();
	}
	
	/**
	 * Initialization code for test mode 
	 */
	@Override
	public void testInit() {
		stopAutonomousRoutine();
		Scheduler.getInstance().run();
	}
	
	/**
	 * Starts the autonomous routine
	 */
	private void startAutonomousRoutine() {
		if(autonomousRoutine != null && !autonomousRoutine.isRunning())
			autonomousRoutine.start();
	}
	
	/**
	 * Stops the autonomous routine
	 */
	private void stopAutonomousRoutine() {
		if(autonomousRoutine != null)
			autonomousRoutine.cancel();
	}
}
