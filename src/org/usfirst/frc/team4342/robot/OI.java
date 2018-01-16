package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.commands.StartClimber;
import org.usfirst.frc.team4342.robot.commands.ElevatePickupCube;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleLow;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.StartIntake;
import org.usfirst.frc.team4342.robot.commands.StartRelease;
import org.usfirst.frc.team4342.robot.commands.StopClimber;
import org.usfirst.frc.team4342.robot.commands.StopIntake;
import org.usfirst.frc.team4342.robot.commands.StopTankDrive;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraightWithJoystick;
import org.usfirst.frc.team4342.robot.commands.TankGoToAngle;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;
import org.usfirst.frc.team4342.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
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
	
	// Subsystems
	public final Intake Intake;
	public final Elevator Elevator;
	public final TankDrive TankDrive;
	public final Climber Climber;
	
	// Motors, sensors, and joysticks
	public final TalonSRX FrontLeft, FrontRight, RearLeft, RearRight,
						  IntakeMotor, ClimberMotor, EleMotor;
	public final AHRS NavX;
	public final Ultrasonic CubeUltra;
	public final Encoder LeftDrive, RightDrive, EleEnc;
	public final Joystick LeftDriveStick, RightDriveStick, SwitchBox, ElevatorStick;
	public final DigitalInput EleLS;
	
	private OI() {
		Logger.info("Constructing the Operator Interface (OI).....");
		
		// Joysticks and Switch Box
		// TODO: Switch to only using the switch box 
		// for elevator and intake once Leo is finished modifying
		// the switch box
		LeftDriveStick = new Joystick(RobotMap.LEFT_DRIVE_STICK);
		RightDriveStick = new Joystick(RobotMap.RIGHT_DRIVE_STICK);
		ElevatorStick = new Joystick(RobotMap.ELEVATOR_STICK);
		SwitchBox = new Joystick(RobotMap.SWITCH_BOX);
		
		// NavX
		NavX = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);
		
		// Ultrasonics
		CubeUltra = new Ultrasonic(RobotMap.CUBE_ULTRA_OUT, RobotMap.CUBE_ULTRA_IN);
		CubeUltra.setAutomaticMode(true);
		
		// TalonSRXs for DriveTrain, Intake, Climber and Elevator
		FrontLeft = new TalonSRX(RobotMap.FRONT_LEFT);
		FrontRight = new TalonSRX(RobotMap.FRONT_RIGHT);
		RearLeft = new TalonSRX(RobotMap.REAR_LEFT);
		RearRight = new TalonSRX(RobotMap.REAR_RIGHT);
		IntakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR);
		ClimberMotor = new TalonSRX(RobotMap.CLIMBER_MOTOR);
		EleMotor = new TalonSRX(RobotMap.ELE_MOTOR);
		
		IntakeMotor.setNeutralMode(NeutralMode.Coast);
		ClimberMotor.setNeutralMode(NeutralMode.Brake);
		EleMotor.setNeutralMode(NeutralMode.Brake);
		
		// Encoders for Drive Train and Elevator
		LeftDrive = new Encoder(RobotMap.LEFT_DRIVE_IN, RobotMap.LEFT_DRIVE_OUT);
		RightDrive = new Encoder(RobotMap.RIGHT_DRIVE_IN, RobotMap.RIGHT_DRIVE_OUT);
		EleEnc = new Encoder(RobotMap.ELE_ENC_IN, RobotMap.ELE_ENC_OUT);
		
		// TODO: Set distance per pulse so encoder values are in inches
		// (Wheel Radius * PI * Num Input Teeth) / (Dist Per Pulse of Encoder * Num Encoder Gear Ratio? * Num Output Teeth)
		LeftDrive.setDistancePerPulse(1);
		RightDrive.setDistancePerPulse(1);
		EleEnc.setDistancePerPulse(1);
		
		// Limit Switch on the bottom of the elevator
		EleLS = new DigitalInput(RobotMap.ELE_LS);

		// Subsystems
		Intake = new Intake(IntakeMotor);
		Elevator = new Elevator(EleMotor, EleEnc, EleLS, CubeUltra);
		TankDrive = new TankDrive(FrontRight, FrontLeft, RearRight, RearLeft, NavX, LeftDrive, RightDrive);
		Climber = new Climber(ClimberMotor);
		
		TankDrive.setNeutralMode(NeutralMode.Coast);

		// Climbing button to enable the winch
		JoystickButton climbButton = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.CLIMB);
		climbButton.whenPressed(new StartClimber(Climber));
		climbButton.whenReleased(new StopClimber(Climber));
		
		// Switch to enable the intake for a cube
		JoystickButton intakeSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.INTAKE);
		intakeSwitch.whenPressed(new StartIntake(Intake));
		intakeSwitch.whenReleased(new StopIntake(Intake));
		
		// Switch to enable reverse intake to release a cube
		JoystickButton releaseSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.RELEASE);
		releaseSwitch.whenPressed(new StartRelease(Intake));
		releaseSwitch.whenReleased(new StopIntake(Intake));
		
		// Button to enable the intake for a cube
		JoystickButton intakeButton = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.INAKE);
		intakeButton.whenPressed(new StartIntake(Intake));
		intakeButton.whenReleased(new StopIntake(Intake));
		
		// Button to enable reverse intake to release a cube
		JoystickButton releaseButton = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.RELEASE);
		releaseButton.whenPressed(new StartRelease(Intake));
		releaseButton.whenReleased(new StopIntake(Intake));
		
		// Button to set the elevator to the scale platform height when it's at its highest point (they have ownership)
		JoystickButton elevateHigh = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_SCALE_HIGH);
		elevateHigh.whenPressed(new ElevateToScaleHigh(Elevator));
		
		// Button to set the elevator to the scale platform height when it's at its neutral point (no one has ownership)
		JoystickButton elevateNeutral = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_SCLALE_NEUTRAL);
		elevateNeutral.whenPressed(new ElevateToScaleNeutral(Elevator));
		
		// Button to set the elevator to the scale when it's at its lowest point (we have ownership)
		JoystickButton elevateLow = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_SCALE_LOW);
		elevateLow.whenPressed(new ElevateToScaleLow(Elevator));
		
		// Button to set the elevator to the switch height
		JoystickButton elevateSwitch = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_SWITCH);
		elevateSwitch.whenPressed(new ElevateToSwitch(Elevator));
		
		// Button to set the elevator to its lowest point to pick up a cube
		JoystickButton elevateCube = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_PICKUP_CUBE);
		elevateCube.whenPressed(new ElevatePickupCube(Elevator));
		
		// Button on the right drive stick to maintain the robot's current heading
		JoystickButton driveStraight = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_STRAIGHT);
		driveStraight.whenPressed(new TankDriveStraightWithJoystick(RightDriveStick, TankDrive));
		driveStraight.whenReleased(new StopTankDrive(TankDrive));
		
		// Button on the right drive stick to go to zero heading (facing towards opponent's alliance wall)
		JoystickButton goToZero = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_ZERO);
		goToZero.whenPressed(new TankGoToAngle(TankDrive, 0));
		
		// Button on the right drive stick to go to -90 degree heading (facing towards left side of the field)
		JoystickButton goToLeft = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_LEFT);
		goToLeft.whenPressed(new TankGoToAngle(TankDrive, -90));
		
		// Button on the right drive stick to go to 90 degree heading (facing towards right side of the field)
		JoystickButton goToRight = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_RIGHT);
		goToRight.whenPressed(new TankGoToAngle(TankDrive, 90));
		
		// Button on the right drive stick to go to 180 degree heading (facing towards our alliance wall)
		JoystickButton go180 = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_180);
		go180.whenPressed(new TankGoToAngle(TankDrive, 180));
	}
}
