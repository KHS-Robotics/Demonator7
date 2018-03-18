package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.commands.climber.StartClimber;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveGoToAngle;
import org.usfirst.frc.team4342.robot.commands.swerve.DriveStraightWithJoystickSwerve;
import org.usfirst.frc.team4342.robot.commands.swerve.SetFOD;
import org.usfirst.frc.team4342.robot.commands.swerve.SetSwerveOverride;
import org.usfirst.frc.team4342.robot.commands.swerve.SwerveSetX;
import org.usfirst.frc.team4342.robot.commands.swerve.SwerveSetY;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevatePickupCube;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToScaleLow;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.elevator.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.elevator.StopElevator;
import org.usfirst.frc.team4342.robot.commands.intake.FastRelease;
import org.usfirst.frc.team4342.robot.commands.intake.SlowRelease;
import org.usfirst.frc.team4342.robot.commands.intake.StartIntake;
import org.usfirst.frc.team4342.robot.commands.intake.StartRelease;
import org.usfirst.frc.team4342.robot.commands.StopSubsystem;
import org.usfirst.frc.team4342.robot.commands.tuning.DrivePIDTuner;
import org.usfirst.frc.team4342.robot.commands.tuning.ElevatorPIDTuner;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive.SwerveModule;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private static OI instance;
	
	/**
	 * Gets the instance of the Operator Interface
	 * @return the instance of the Operator Interface
	 */
	public static OI getInstance() {
		if(instance == null)
			instance = new OI();
		
		return instance;
	}

	// PowerDistributionPanel
	public PowerDistributionPanel PDP;

	// Joysticks
	public XboxController DriveController;
	public Joystick SwitchBox;

	// Drive
	public SwerveDrive Drive;
	private SwerveModule FR, FL, RR, RL;
	private AHRS NavX;
	private TalonSRX FrontRightDrive, FrontLeftDrive, RearLeftDrive, RearRightDrive;
	private TalonSRX FrontRightPivot, FrontLeftPivot, RearRightPivot, RearLeftPivot;
	private Encoder FrontRightDriveEnc, FrontLeftDriveEnc, RearRightDriveEnc, RearLeftDriveEnc;
	private AnalogInput FrontRightPivotEnc, FrontLeftPivotEnc, RearRightPivotEnc, RearLeftPivotEnc;
	private Ultrasonic CubeUltra;

	// Climber
	public Climber Climber;
	private Talon ClimberMotor1, ClimberMotor2;

	// Elevator
	public Elevator Elevator;
	private TalonSRX EleMotor;
	private Encoder EleEnc;
	private DigitalInput EleLS;
	
	// Intake
	public Intake Intake;
	private Talon IntakeMotorLeft, IntakeMotorRight;
	
	private OI() {
		Logger.info("Connecting to PDP...");

		// Power Distribution Panel
		PDP = new PowerDistributionPanel();
		LiveWindow.disableTelemetry(PDP);

		Logger.info("Connecting Xbox Controller and Switch Box...");

		// Joysticks
		DriveController = new XboxController(RobotMap.XBOX_PORT);
		SwitchBox = new Joystick(RobotMap.SWITCH_BOX);
		SwitchBox.setTwistChannel(3); // twist channel for y input for right thumbstick

		// Subsystems
		initDrive();
		initIntake();
		initClimber();
		initElevator();

		// Button to tune PID via SmartDashboard
//		JoystickButton tunePID = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.TUNE_PID);
//		if(Drive != null) {
//			//initDrivePIDTuner(tunePID);
//		}
//		if(Elevator != null) {
//			//initElevatorPIDTuner(tunePID);
//		}
	}

	/**
	 * Initializes the Drive Train
	 */
	private void initDrive() {
		try {
			Logger.info("Initializing Swerve...");

			// NavX Board
			NavX = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);

			// Drive Motors
			FrontRightDrive = new TalonSRX(RobotMap.FRONT_RIGHT_DRIVE);
			FrontLeftDrive = new TalonSRX(RobotMap.FRONT_LEFT_DRIVE);
			RearRightDrive = new TalonSRX(RobotMap.REAR_RIGHT_DRIVE);
			RearLeftDrive = new TalonSRX(RobotMap.REAR_LEFT_DRIVE);

			// Pivot Motors
			FrontRightPivot = new TalonSRX(RobotMap.FRONT_RIGHT_PIVOT);
			FrontLeftPivot = new TalonSRX(RobotMap.FRONT_LEFT_PIVOT);
			RearRightPivot = new TalonSRX(RobotMap.REAR_RIGHT_PIVOT);
			RearLeftPivot = new TalonSRX(RobotMap.REAR_LEFT_PIVOT);

			// Drive/Traslational Encoders
			FrontRightDriveEnc = new Encoder(RobotMap.FRONT_RIGHT_DRIVE_ENC_A, RobotMap.FRONT_RIGHT_DRIVE_ENC_B);
			FrontLeftDriveEnc = new Encoder(RobotMap.FRONT_LEFT_DRIVE_ENC_A, RobotMap.FRONT_LEFT_DRIVE_ENC_B);
			RearRightDriveEnc = new Encoder(RobotMap.REAR_RIGHT_DRIVE_ENC_A, RobotMap.REAR_RIGHT_DRIVE_ENC_B);
			RearLeftDriveEnc = new Encoder(RobotMap.REAR_LEFT_DRIVE_ENC_A, RobotMap.REAR_LEFT_DRIVE_ENC_B);

			// (18 input teeth * 4 inch diameter wheels * PI) / (2048 pulses per rev * 54 output teeth)
//			final double distancePerPulse = ((4*Math.PI * (18) / (256 )); //when tested, off by a factor of 4.5   18*4*Math.PI) / (2048*32
			final double distancePerPulse = ((4*Math.PI * 12 * 19) / (2048 * 32 * 60)); //* (19/60) * (12/32);
			FrontRightDriveEnc.setDistancePerPulse(distancePerPulse);
			FrontLeftDriveEnc.setDistancePerPulse(distancePerPulse);
			RearRightDriveEnc.setDistancePerPulse(distancePerPulse);
			RearLeftDriveEnc.setDistancePerPulse(distancePerPulse);

			// Pivot/Rotational analog inputs
			FrontRightPivotEnc = new AnalogInput(RobotMap.FRONT_RIGHT_PIVOT_CHANNEL);
			FrontLeftPivotEnc = new AnalogInput(RobotMap.FRONT_LEFT_PIVOT_CHANNEL);
			RearRightPivotEnc = new AnalogInput(RobotMap.REAR_RIGHT_PIVOT_CHANNEL);
			RearLeftPivotEnc = new AnalogInput(RobotMap.REAR_LEFT_PIVOT_CHANNEL);

			FR = new SwerveModule(
				FrontRightDrive, 
				FrontRightDriveEnc, 
				FrontRightPivot, 
				FrontRightPivotEnc, 
				Constants.Drive.PivotPID.FR_P, 
				Constants.Drive.PivotPID.FR_I, 
				Constants.Drive.PivotPID.FR_D
			);

			FL = new SwerveModule(
				FrontLeftDrive, 
				FrontLeftDriveEnc, 
				FrontLeftPivot, 
				FrontLeftPivotEnc, 
				Constants.Drive.PivotPID.FL_P, 
				Constants.Drive.PivotPID.FL_I, 
				Constants.Drive.PivotPID.FL_D
			);
			FL.setReverse(true);

			RR = new SwerveModule(
				RearRightDrive, 
				RearRightDriveEnc, 
				RearRightPivot, 
				RearRightPivotEnc, 
				Constants.Drive.PivotPID.RR_P, 
				Constants.Drive.PivotPID.RR_I, 
				Constants.Drive.PivotPID.RR_D
			);

			RL = new SwerveModule(
				RearLeftDrive, 
				RearLeftDriveEnc, 
				RearLeftPivot, 
				RearLeftPivotEnc, 
				Constants.Drive.PivotPID.RL_P, 
				Constants.Drive.PivotPID.RL_I, 
				Constants.Drive.PivotPID.RL_D
			);
			RL.setReverse(true);
			
			// Swerve
			Drive = new SwerveDrive(FR, FL, RR, RL, NavX);
			
			FL.setOffset(0);
			FR.setOffset(0);
			RL.setOffset(0);
			RR.setOffset(0);
			
			Drive.setNeutralMode(NeutralMode.Brake);
			
			// Button on the right drive stick to go to zero heading (facing towards opponent's alliance wall)
			JoystickButton goToZero = new JoystickButton(DriveController, ButtonMap.DriveController.GO_TO_ZERO);
			goToZero.whenPressed(new DriveGoToAngle(Drive, 0));
			
			// Button on the right drive stick to go to -90 degree heading (facing towards left side of the field)
			JoystickButton goToLeft = new JoystickButton(DriveController, ButtonMap.DriveController.GO_TO_LEFT);
			goToLeft.whenPressed(new DriveGoToAngle(Drive, -90));
			
			// Button on the right drive stick to go to 90 degree heading (facing towards right side of the field)
			JoystickButton goToRight = new JoystickButton(DriveController, ButtonMap.DriveController.GO_TO_RIGHT);
			goToRight.whenPressed(new DriveGoToAngle(Drive, 90));
			
			// Button on the right drive stick to go to 180 degree heading (facing towards our alliance wall)
			JoystickButton go180 = new JoystickButton(DriveController, ButtonMap.DriveController.GO_TO_180);
			go180.whenPressed(new DriveGoToAngle(Drive, 180));
			
			JoystickButton setFOD = new JoystickButton(DriveController, ButtonMap.DriveController.ROBOT_ORIENT);
			setFOD.whenPressed(new SetFOD(Drive, true));
			setFOD.whenReleased(new SetFOD(Drive, false));
			
//			JoystickButton test = new JoystickButton(DriveController, 7);
//			test.whenPressed(new SwerveSetY(Drive, false));
//			test.whenReleased(new StopSubsystem(Drive));
			
		} catch(Exception ex) {
			Logger.error("Failed to initialize Drive!", ex);
		}
	}

	/**
	 * Initializes the Climber
	 */
	private void initClimber() {
		try {
			Logger.info("Initializing Climber...");

			// Climber
			ClimberMotor1 = new Talon(RobotMap.CLIMBER_MOTOR);
			ClimberMotor2 = new Talon(RobotMap.CLIMBER_MOTOR_2);
//			ClimberMotor1.setNeutralMode(NeutralMode.Brake);
//			ClimberMotor2.setNeutralMode(NeutralMode.Brake);
			Climber = new Climber(ClimberMotor1, ClimberMotor2);

			// Climbing button to enable the winch
			JoystickButton climbButton = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.CLIMB);
			climbButton.whenPressed(new StartClimber(Climber));
			climbButton.whenReleased(new StopSubsystem(Climber));
			
			// Back up climb button in case other button breaks
			// switch it opposite
			
			JoystickButton backupClimbButton = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.BACKUP_CLIMB);
			backupClimbButton.whenReleased(new StartClimber(Climber));
			backupClimbButton.whenPressed(new StopSubsystem(Climber));
		} catch(Exception ex) {
			Logger.error("Failed to initialize Climber!", ex);
		}
	}
	
	/**
	 * Initializes the Elevator
	 */
	private void initElevator() {
		try {
			Logger.info("Initializing Elevator...");
			
			// Elevator
			EleMotor = new TalonSRX(RobotMap.ELE_MOTOR);
			EleMotor.setNeutralMode(NeutralMode.Brake);
			EleEnc = new Encoder(RobotMap.ELE_ENC_A, RobotMap.ELE_ENC_B);
			EleEnc.setDistancePerPulse(1);
			EleLS = new DigitalInput(RobotMap.ELE_LS);
			Elevator = new Elevator(EleMotor, EleEnc, EleLS);
			
			// Button to override driving slow when elevator is raised high enough
			JoystickButton slowOverride = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.DRIVE_SLOW_OVERRIDE);
			slowOverride.whenPressed(new SetSwerveOverride(Drive, true));
			slowOverride.whenPressed(new SetSwerveOverride(Drive, false));

			// Button to set the elevator to the scale platform height when it's at its highest point (they have ownership)
			//JoystickButton elevateHigh = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_SCALE_HIGH);
			//elevateHigh.whenPressed(new ElevateToScaleHigh(Elevator));
			//elevateHigh.whenReleased(new StopElevator(Elevator));

			// Button to set the elevator to the scale platform height when it's at its neutral point (no one has ownership)
			//JoystickButton elevateNeutral = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_SCLALE_NEUTRAL);
			//elevateNeutral.whenPressed(new ElevateToScaleNeutral(Elevator));
			//elevateNeutral.whenReleased(new StopElevator(Elevator));
			
			// Button to set the elevator to the scale when it's at its lowest point (we have ownership)
			//JoystickButton elevateLow = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_SCALE_LOW);
			//elevateLow.whenPressed(new ElevateToScaleLow(Elevator));
			//elevateLow.whenReleased(new StopElevator(Elevator));
			
			// Button to set the elevator to the switch height
			// Switch is opposite
			//JoystickButton elevateSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_SWITCH);
			//elevateSwitch.whenReleased(new ElevateToSwitch(Elevator));
			//elevateSwitch.whenPressed(new StopElevator(Elevator));
			
			// Button to set the elevator to its lowest point to pick up a cube
			// Switch is opposite
			JoystickButton elevateCube = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_PICKUP_CUBE);
			elevateCube.whenReleased(new ElevatePickupCube(Elevator));
			elevateCube.whenPressed(new StopElevator(Elevator));
		} catch(Exception ex) {
			Logger.error("Failed to initialize Elevator!", ex);
		}
	}

	/**
	 * Initializes the Intake
	 */
	private void initIntake() {
		try {
			Logger.info("Initializing Intake...");
			
			// Intake
			IntakeMotorLeft = new Talon(RobotMap.INTAKE_MOTOR_LEFT);
			IntakeMotorRight = new Talon(RobotMap.INTAKE_MOTOR_RIGHT);
//			CubeUltra = new Ultrasonic(RobotMap.ULTRA_OUT, RobotMap.ULTRA_IN);
//			CubeUltra.setAutomaticMode(true);
			Intake = new Intake(IntakeMotorLeft, IntakeMotorRight, CubeUltra);

			// Switch to enable the intake for a cube
			JoystickButton intakeSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.INTAKE);
			intakeSwitch.whenPressed(new StartIntake(Intake));
			intakeSwitch.whenReleased(new StopSubsystem(Intake));
			
			// Switch to enable reverse intake to release a cube
			JoystickButton slowReleaseSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.SLOW_RELEASE);
			slowReleaseSwitch.whenPressed(new SlowRelease(Intake));
			slowReleaseSwitch.whenReleased(new FastRelease(Intake));
						
			// Switch to enable reverse intake to release a cube
			JoystickButton releaseSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.RELEASE);
			releaseSwitch.whenPressed(new StartRelease(Intake));
			releaseSwitch.whenReleased(new StopSubsystem(Intake));
			
		} catch(Exception ex) {
			Logger.error("Failed to initialize Intake!", ex);
		}
	}

	/**
	 * Initializes the Drive PID Tuner
	 */
	private void initDrivePIDTuner(JoystickButton button) {
		try {
			Logger.info("Initializing Drive PID Tuner...");

			// Switch is opposite
			button.whenReleased(new DrivePIDTuner(Drive));
//			button.whenReleased(new PivotPIDTuner(FR, "FR", 
//				Constants.Drive.PivotPID.FR_P, 
//				Constants.Drive.PivotPID.FR_D, 
//				Constants.Drive.PivotPID.FR_D)
//			);
//			button.whenReleased(new PivotPIDTuner(FL, "FL", 
//				Constants.Drive.PivotPID.FL_P, 
//				Constants.Drive.PivotPID.FL_D, 
//				Constants.Drive.PivotPID.FL_D)
//			);
//			button.whenReleased(new PivotPIDTuner(RR, "RR", 
//				Constants.Drive.PivotPID.RR_P, 
//				Constants.Drive.PivotPID.RR_D, 
//				Constants.Drive.PivotPID.RR_D)
//			);
//			button.whenReleased(new PivotPIDTuner(RL, "RL", 
//				Constants.Drive.PivotPID.RL_P, 
//				Constants.Drive.PivotPID.RL_D, 
//				Constants.Drive.PivotPID.RL_D)
//			);
			button.whenPressed(new StopSubsystem(Drive));
//			button.whenPressed(new StopSubsystem(FR));
//			button.whenPressed(new StopSubsystem(FL));
//			button.whenPressed(new StopSubsystem(RR));
//			button.whenPressed(new StopSubsystem(RL));
		} catch(Exception ex) {
			Logger.error("Failed to initialize Drive PID Tuner!", ex);
		}
	}

	/**
	 * Initializes the Elevator PID Tuner
	 */
	private void initElevatorPIDTuner(JoystickButton button) {
		try {
			Logger.info("Initializing Elevator PID Tuner...");
			
			button.whenReleased(new ElevatorPIDTuner(Elevator));
			button.whenPressed(new StopElevator(Elevator));
		} catch(Exception ex) {
			Logger.error("Failed to initialize Elevator PID Tuner!", ex);
		}
	}
}
