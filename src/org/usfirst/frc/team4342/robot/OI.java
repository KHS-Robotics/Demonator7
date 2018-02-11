package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.commands.StartClimber;
import org.usfirst.frc.team4342.robot.commands.DriveGoToAngle;
import org.usfirst.frc.team4342.robot.commands.DriveStraightWithJoystick;
import org.usfirst.frc.team4342.robot.commands.DriveStraightWithJoystickSwerve;
import org.usfirst.frc.team4342.robot.commands.ElevatePickupCube;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleLow;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.StartIntake;
import org.usfirst.frc.team4342.robot.commands.StartRelease;
import org.usfirst.frc.team4342.robot.commands.StopElevator;
import org.usfirst.frc.team4342.robot.commands.StopSubsystem;
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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


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
	public SwerveModule FR, FL, RR, RL;
	public AHRS NavX;
	public TalonSRX FrontRightDrive, FrontLeftDrive, RearLeftDrive, RearRightDrive;
	public TalonSRX FrontRightPivot, FrontLeftPivot, RearRightPivot, RearLeftPivot;
	public Encoder FrontRightDriveEnc, FrontLeftDriveEnc, RearRightDriveEnc, RearLeftDriveEnc;
	public AnalogInput FrontRightPivotEnc, FrontLeftPivotEnc, RearRightPivotEnc, RearLeftPivotEnc;

	// Climber
	public Climber Climber;
	public Spark ClimberMotor;

	// Elevator
	public Elevator Elevator;
	public TalonSRX EleMotor;
	public Encoder EleEnc;
	public DigitalInput EleLS;
	
	// Intake
	public Intake Intake;
	public Spark IntakeMotor;
	
	private OI() {
		Logger.info("Connecting to PDP...");

		// Power Distribution Panel
		PDP = new PowerDistributionPanel();

		Logger.info("Initializing joysticks...");
		// Joysticks
		// Xbox Controller
		DriveController = new XboxController(RobotMap.XBOX_PORT);
		// Switch Box
		SwitchBox = new Joystick(RobotMap.SWITCH_BOX);
		SwitchBox.setTwistChannel(3); // twist channel for y input for right thumbstick

		initDrive();
		initClimber();
		initElevator();
		initIntake();
	}

	/**
	 * Initializes the Drive Train
	 */
	private void initDrive() {
		Logger.info("Initializing Swerve...");

		// NavX Board
		NavX = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);

		// Drive Motors
		FrontRightDrive = new TalonSRX(RobotMap.FRONT_RIGHT_DRIVE);
		FrontLeftDrive = new TalonSRX(RobotMap.FRONT_LEFT_DRIVE);
		RearLeftDrive = new TalonSRX(RobotMap.REAR_LEFT_DRIVE);
		RearRightDrive = new TalonSRX(RobotMap.REAR_RIGHT_DRIVE);

		// Pivot Motors
		FrontRightPivot = new TalonSRX(RobotMap.FRONT_RIGHT_PIVOT);
		FrontLeftPivot = new TalonSRX(RobotMap.FRONT_LEFT_PIVOT);
		RearLeftPivot = new TalonSRX(RobotMap.REAR_LEFT_PIVOT);
		RearRightPivot = new TalonSRX(RobotMap.REAR_RIGHT_PIVOT);

		// Drive/Traslational Encoders
		FrontRightDriveEnc = new Encoder(RobotMap.FRONT_LEFT_DRIVE_ENC_A, RobotMap.FRONT_LEFT_DRIVE_ENC_B);
		FrontLeftDriveEnc = new Encoder(RobotMap.FRONT_LEFT_DRIVE_ENC_A, RobotMap.FRONT_LEFT_DRIVE_ENC_B);
		RearRightDriveEnc = new Encoder(RobotMap.REAR_RIGHT_DRIVE_ENC_A, RobotMap.REAR_RIGHT_DRIVE_ENC_B);
		RearLeftDriveEnc = new Encoder(RobotMap.REAR_LEFT_DRIVE_ENC_A, RobotMap.REAR_LEFT_DRIVE_ENC_B);

		final double distancePerPulse = 2048*(18.0/54.0)*(Math.PI*6);
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

		// Swerve
		Drive = new SwerveDrive(FR, FL, RR, RL, NavX);
		Drive.setNeutralMode(NeutralMode.Brake);
		// Drive.setFieldOriented(true);

		// Button to maintain heading and move forward/backward
		JoystickButton driveStraight = new JoystickButton(DriveController, ButtonMap.DriveController.GO_STRAIGHT_Y);
		driveStraight.whenPressed(new DriveStraightWithJoystick(DriveController, Drive));
		driveStraight.whenReleased(new StopSubsystem(Drive));

		// Button to maintain heading and strafe
		JoystickButton strafeStraight = new JoystickButton(DriveController, ButtonMap.DriveController.GO_STRAIGHT_X);
		strafeStraight.whenPressed(new DriveStraightWithJoystickSwerve(DriveController, Drive, true));
		strafeStraight.whenReleased(new StopSubsystem(Drive));
		
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
	}

	/**
	 * Initializes the Climber
	 */
	private void initClimber() {
		Logger.info("Initializing Climber...");

		// Climber
		ClimberMotor = new Spark(RobotMap.CLIMBER_MOTOR);
		Climber = new Climber(ClimberMotor);

		// Climbing button to enable the winch
		// Switch is opposite
		JoystickButton climbButton = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.CLIMB);
		climbButton.whenPressed(new StopSubsystem(Climber));
		climbButton.whenReleased(new StartClimber(Climber));
	}
	
	/**
	 * Initializes the Elevator
	 */
	private void initElevator() {
		Logger.info("Initializing Elevator...");
		
		// Elevator
		EleMotor = new TalonSRX(RobotMap.ELE_MOTOR);
		EleEnc = new Encoder(RobotMap.ELE_ENC_IN, RobotMap.ELE_ENC_OUT);
		// TODO: Set distance per pulse for elevator encoder
		EleEnc.setDistancePerPulse(1);
		EleLS = new DigitalInput(RobotMap.ELE_LS);
		Elevator = new Elevator(EleMotor, EleEnc, EleLS);
		Elevator.setNeutralMode(NeutralMode.Brake);

		// Button to set the elevator to the scale platform height when it's at its highest point (they have ownership)
		JoystickButton elevateHigh = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_SCALE_HIGH);
		elevateHigh.whenPressed(new ElevateToScaleHigh(Elevator));
		elevateHigh.whenReleased(new StopElevator(Elevator));

		// Button to set the elevator to the scale platform height when it's at its neutral point (no one has ownership)
		JoystickButton elevateNeutral = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_SCLALE_NEUTRAL);
		elevateNeutral.whenPressed(new ElevateToScaleNeutral(Elevator));
		elevateNeutral.whenReleased(new StopElevator(Elevator));
		
		// Button to set the elevator to the scale when it's at its lowest point (we have ownership)
		JoystickButton elevateLow = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_SCALE_LOW);
		elevateLow.whenPressed(new ElevateToScaleLow(Elevator));
		elevateLow.whenReleased(new StopElevator(Elevator));
		
		// Button to set the elevator to the switch height
		// Switch is opposite
		JoystickButton elevateSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_SWITCH);
		elevateSwitch.whenReleased(new ElevateToSwitch(Elevator));
		elevateSwitch.whenPressed(new StopElevator(Elevator));
		
		// Button to set the elevator to its lowest point to pick up a cube
		// Switch is opposite
		JoystickButton elevateCube = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.ELEVATE_PICKUP_CUBE);
		elevateCube.whenReleased(new ElevatePickupCube(Elevator));
		elevateCube.whenPressed(new StopElevator(Elevator));
	}

	/**
	 * Initializes the Intake
	 */
	private void initIntake() {
		Logger.info("Initializing Intake...");
		
		// Intake
		IntakeMotor = new Spark(RobotMap.INTAKE_MOTOR);
		Intake = new Intake(IntakeMotor);

		// Switch to enable the intake for a cube
		JoystickButton intakeSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.INTAKE);
		intakeSwitch.whenPressed(new StartIntake(Intake));
		intakeSwitch.whenReleased(new StopSubsystem(Intake));
		
		// Switch to enable reverse intake to release a cube
		JoystickButton releaseSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.RELEASE);
		releaseSwitch.whenPressed(new StartRelease(Intake));
		releaseSwitch.whenReleased(new StopSubsystem(Intake));
	}
}
