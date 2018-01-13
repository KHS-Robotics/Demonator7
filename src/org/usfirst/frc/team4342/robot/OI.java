package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.commands.StartClimber;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleLow;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.StartIntake;
import org.usfirst.frc.team4342.robot.commands.ReleaseCube;
import org.usfirst.frc.team4342.robot.commands.StopClimber;
import org.usfirst.frc.team4342.robot.commands.StopIntake;
import org.usfirst.frc.team4342.robot.commands.SwerveDriveStraight;
import org.usfirst.frc.team4342.robot.commands.TankDriveStraightDistance;
import org.usfirst.frc.team4342.robot.logging.Logger;
import org.usfirst.frc.team4342.robot.subsystems.Intake;
import org.usfirst.frc.team4342.robot.subsystems.Elevator;
import org.usfirst.frc.team4342.robot.subsystems.TankDrive;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive;
import org.usfirst.frc.team4342.robot.subsystems.Climber;
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
	public final SwerveDrive SwerveDrive;
	public final Climber Climber;
	
	public final TalonSRX FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight,
						  IntakeMotor, ClimberMotor, EleMotor;
	public final AHRS NavX;
	public final Ultrasonic LeftHeight, RightHeight, LeftDistance, RightDistance;
	public final Encoder LeftDrive, RightDrive, EleEnc;
	public final Joystick LeftDriveStick, RightDriveStick, SwitchBox;
	public final DigitalInput EleLS;
	
	private OI() {
		Logger.info("Constructing OI.....");
		
		// Joysticks
		LeftDriveStick = new Joystick(RobotMap.LEFT_DRIVE_STICK);
		RightDriveStick = new Joystick(RobotMap.RIGHT_DRIVE_STICK);
		SwitchBox = new Joystick(RobotMap.SWITCH_BOX);
		
		// NavX
		NavX = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);
		
		// Ultrasonics
		LeftHeight = new Ultrasonic(RobotMap.LEFT_HEIGHT_IN, RobotMap.LEFT_HEIGHT_IN);
		RightHeight = new Ultrasonic(RobotMap.RIGHT_HEIGHT_IN, RobotMap.RIGHT_HEIGHT_OUT);
		LeftDistance = new Ultrasonic(RobotMap.LEFT_DISTANCE_IN,RobotMap.LEFT_DISTANCE_OUT);
		RightDistance = new Ultrasonic(RobotMap.RIGHT_DISTANCE_IN, RobotMap.RIGHT_DISTANCE_OUT);
		
		LeftHeight.setAutomaticMode(true);
		RightHeight.setAutomaticMode(true);
		LeftDistance.setAutomaticMode(true);
		RightDistance.setAutomaticMode(true);
		
		// TalonSRXs
		FrontLeft = new TalonSRX(RobotMap.FRONT_LEFT);
		FrontRight = new TalonSRX(RobotMap.FRONT_RIGHT);
		RearLeft = new TalonSRX(RobotMap.REAR_LEFT);
		RearRight = new TalonSRX(RobotMap.REAR_RIGHT);
		MiddleLeft = new TalonSRX(RobotMap.MIDDLE_LEFT);
		MiddleRight = new TalonSRX(RobotMap.MIDDLE_RIGHT);
		IntakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR);
		ClimberMotor = new TalonSRX(RobotMap.CLIMBER_MOTOR);
		EleMotor = new TalonSRX(RobotMap.ELE_MOTOR);
		
		// Encoders
		LeftDrive = new Encoder(RobotMap.LEFT_DRIVE_IN, RobotMap.LEFT_DRIVE_OUT);
		RightDrive = new Encoder(RobotMap.RIGHT_DRIVE_IN, RobotMap.RIGHT_DRIVE_OUT);
		EleEnc = new Encoder(RobotMap.ELE_ENC_IN, RobotMap.ELE_ENC_OUT);
		
		EleLS = new DigitalInput(RobotMap.ELE_LS);
		
		SwerveDrive = new SwerveDrive(FrontRight, FrontLeft, RearRight, RearLeft, LeftDrive, RightDrive, NavX);

		// Subsystems
		Intake = new Intake(IntakeMotor);
		Elevator = new Elevator(EleMotor, EleEnc, EleLS);
		TankDrive = new TankDrive(FrontRight, FrontLeft, MiddleRight, MiddleLeft, RearRight, RearLeft, NavX, LeftDrive, RightDrive);
		Climber = new Climber(ClimberMotor);

		JoystickButton climbButton = new JoystickButton(SwitchBox, 0);
		climbButton.whenPressed(new StartClimber(Climber));
		climbButton.whenReleased(new StopClimber(Climber));
		
		JoystickButton intakeButton = new JoystickButton(SwitchBox, 0);
		intakeButton.whenPressed(new StartIntake(Intake));
		intakeButton.whenReleased(new StopIntake(Intake));
		
		JoystickButton placeButton = new JoystickButton(SwitchBox, 0);
		placeButton.whenPressed(new ReleaseCube(Intake));
		
		JoystickButton elevateHigh = new JoystickButton(SwitchBox, 0);
		elevateHigh.whenPressed(new ElevateToScaleHigh(Elevator));
		
		JoystickButton elevateNeutral = new JoystickButton(SwitchBox, 0);
		elevateNeutral.whenPressed(new ElevateToScaleNeutral(Elevator));
		
		JoystickButton elevateLow = new JoystickButton(SwitchBox, 0);
		elevateLow.whenPressed(new ElevateToScaleLow(Elevator));
		
		JoystickButton elevateSwitch = new JoystickButton(SwitchBox, 0);
		elevateSwitch.whenPressed(new ElevateToSwitch(Elevator));
		
		JoystickButton tankDriveStraight = new JoystickButton(LeftDriveStick, 0);
		
		
		JoystickButton swerveDriveStraight = new JoystickButton(LeftDriveStick, 0);
		swerveDriveStraight.whenPressed(new SwerveDriveStraight());
	}
}
