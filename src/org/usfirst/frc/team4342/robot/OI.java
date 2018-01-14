package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.commands.StartClimber;
import org.usfirst.frc.team4342.robot.commands.ElevatePickupCube;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleLow;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.StartIntake;
import org.usfirst.frc.team4342.robot.commands.StartRelease;
import org.usfirst.frc.team4342.robot.commands.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.StopClimber;
import org.usfirst.frc.team4342.robot.commands.StopIntake;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraight;
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
	
	public static OI getInstance() {
		if(instance == null)
			instance = new OI();
		
		return instance;
	}
	
	public final Intake Intake;
	public final Elevator Elevator;
	public final TankDrive TankDrive;
	public final Climber Climber;
	
	public final TalonSRX FrontLeft, FrontRight, RearLeft, RearRight,
						  IntakeMotor, ClimberMotor, EleMotor;
	public final AHRS NavX;
	public final Ultrasonic LeftHeight, RightHeight, LeftDistance, RightDistance;
	public final Encoder LeftDrive, RightDrive, EleEnc;
	public final Joystick LeftDriveStick, RightDriveStick, SwitchBox, ElevatorStick;
	public final DigitalInput EleLS;
	
	private OI() {
		Logger.info("Constructing the Operator Interface (OI).....");
		
		// Joysticks
		LeftDriveStick = new Joystick(RobotMap.LEFT_DRIVE_STICK);
		RightDriveStick = new Joystick(RobotMap.RIGHT_DRIVE_STICK);
		SwitchBox = new Joystick(RobotMap.SWITCH_BOX);
		ElevatorStick = new Joystick(RobotMap.ELEVATOR_STICK);
		
		// NavX
		NavX = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);
		
		// Ultrasonics
		LeftHeight = new Ultrasonic(RobotMap.LEFT_HEIGHT_OUT, RobotMap.LEFT_HEIGHT_IN);
		RightHeight = new Ultrasonic(RobotMap.RIGHT_HEIGHT_OUT, RobotMap.RIGHT_HEIGHT_IN);
		LeftDistance = new Ultrasonic(RobotMap.LEFT_DISTANCE_OUT,RobotMap.LEFT_DISTANCE_IN);
		RightDistance = new Ultrasonic(RobotMap.RIGHT_DISTANCE_OUT, RobotMap.RIGHT_DISTANCE_IN);
		
		LeftHeight.setAutomaticMode(true);
		RightHeight.setAutomaticMode(true);
		LeftDistance.setAutomaticMode(true);
		RightDistance.setAutomaticMode(true);
		
		// TalonSRXs
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
		
		// Encoders
		LeftDrive = new Encoder(RobotMap.LEFT_DRIVE_IN, RobotMap.LEFT_DRIVE_OUT);
		RightDrive = new Encoder(RobotMap.RIGHT_DRIVE_IN, RobotMap.RIGHT_DRIVE_OUT);
		EleEnc = new Encoder(RobotMap.ELE_ENC_IN, RobotMap.ELE_ENC_OUT);
		
		// TODO: Set distance per pulse so encoder values are in inches
		// (Wheel Radius * PI * Num Input Teeth) / (Dist Per Pulse of Encoder * Num Encoder Gear Ratio? * Num Output Teeth)
		LeftDrive.setDistancePerPulse(1);
		RightDrive.setDistancePerPulse(1);
		EleEnc.setDistancePerPulse(1);
		
		EleLS = new DigitalInput(RobotMap.ELE_LS);

		// Subsystems
		Intake = new Intake(IntakeMotor);
		Elevator = new Elevator(EleMotor, EleEnc, EleLS);
		TankDrive = new TankDrive(FrontRight, FrontLeft, RearRight, RearLeft, NavX, LeftDrive, RightDrive);
		Climber = new Climber(ClimberMotor);
		
		TankDrive.setNeutralMode(NeutralMode.Coast);

		JoystickButton climbButton = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.CLIMB);
		climbButton.whenPressed(new StartClimber(Climber));
		climbButton.whenReleased(new StopClimber(Climber));
		
		JoystickButton intakeButton = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.INTAKE);
		intakeButton.whenPressed(new StartIntake(Intake));
		intakeButton.whenReleased(new StopIntake(Intake));
		
		JoystickButton releaseButton = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.RELEASE);
		releaseButton.whenPressed(new ReleaseCube(Intake));
		releaseButton.whenReleased(new StopIntake(Intake));
		
		JoystickButton intakeWithStick = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.INAKE);
		intakeWithStick.whenPressed(new StartIntake(Intake));
		intakeWithStick.whenReleased(new StopIntake(Intake));
		
		JoystickButton releaseWithStick = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.RELEASE);
		releaseWithStick.whenPressed(new StartRelease(Intake));
		releaseWithStick.whenReleased(new StopIntake(Intake));
		
		JoystickButton elevateHigh = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_SCALE_HIGH);
		elevateHigh.whenPressed(new ElevateToScaleHigh(Elevator));
		
		JoystickButton elevateNeutral = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_SCLALE_NEUTRAL);
		elevateNeutral.whenPressed(new ElevateToScaleNeutral(Elevator));
		
		JoystickButton elevateLow = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_SCALE_LOW);
		elevateLow.whenPressed(new ElevateToScaleLow(Elevator));
		
		JoystickButton elevateSwitch = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_SWITCH);
		elevateSwitch.whenPressed(new ElevateToSwitch(Elevator));
		
		JoystickButton elevateCube = new JoystickButton(ElevatorStick, ButtonMap.ElevatorStick.ELEVATE_PICKUP_CUBE);
		elevateCube.whenPressed(new ElevatePickupCube(Elevator));
		
		JoystickButton driveStraight = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_STRAIGHT);
		driveStraight.whenPressed(new TankDriveStraight(RightDriveStick, TankDrive));
		
		JoystickButton goToZero = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_ZERO);
		goToZero.whenPressed(new TankGoToAngle(TankDrive, 0));
		
		JoystickButton goToLeft = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_LEFT);
		goToLeft.whenPressed(new TankGoToAngle(TankDrive, -90));
		
		JoystickButton goToRight = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_RIGHT);
		goToRight.whenPressed(new TankGoToAngle(TankDrive, 90));
		
		JoystickButton go180 = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_180);
		go180.whenPressed(new TankGoToAngle(TankDrive, 180));
	}
}
