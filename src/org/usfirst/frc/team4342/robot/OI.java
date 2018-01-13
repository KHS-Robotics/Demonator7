package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.commands.StartClimber;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleLow;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.StartIntake;
import org.usfirst.frc.team4342.robot.commands.PlaceCube;
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
						  FrontLeftPivot, FrontRightPivot, RearLeftPivot, RearRightPivot,
						  IntakeMotor, ClimberMotor, EleMotor;
	public final AHRS NavX;
	public final Ultrasonic LeftHeight, RightHeight, LeftDistance, RightDistance;
	public final Encoder LeftDrive, RightDrive, FrontLeftEnc, FrontRightEnc, RearLeftEnc, RearRightEnc, EleEnc;
	public final Joystick LeftDriveStick, RightDriveStick, SwitchBox;
	public final DigitalInput EleLS;
	
	private OI() {
		Logger.info("Constructing OI.....");
		
		// Joysticks
		LeftDriveStick = new Joystick(0);
		RightDriveStick = new Joystick(1);
		SwitchBox = new Joystick(2);
		
		// NavX
		NavX = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);
		
		// Ultrasonics
		LeftHeight = new Ultrasonic(0,0);
		RightHeight = new Ultrasonic(0,0);
		LeftDistance = new Ultrasonic(0,0);
		RightDistance = new Ultrasonic(0,0);
		
		LeftHeight.setAutomaticMode(true);
		RightHeight.setAutomaticMode(true);
		LeftDistance.setAutomaticMode(true);
		RightDistance.setAutomaticMode(true);
		
		// TalonSRXs
		FrontLeft = new TalonSRX(0);
		FrontRight = new TalonSRX(0);
		RearLeft = new TalonSRX(0);
		RearRight = new TalonSRX(0);
		MiddleLeft = new TalonSRX(0);
		MiddleRight = new TalonSRX(0);
		FrontLeftPivot = new TalonSRX(0);
		FrontRightPivot = new TalonSRX(0);
		RearLeftPivot = new TalonSRX(0);
		RearRightPivot = new TalonSRX(0);
		IntakeMotor = new TalonSRX(0);
		ClimberMotor = new TalonSRX(0);
		EleMotor = new TalonSRX(0);
		
		// Encoders
		LeftDrive = new Encoder(0,0);
		RightDrive = new Encoder(0,0);
		FrontLeftEnc = new Encoder(0,0);
		FrontRightEnc = new Encoder(0,0);
		RearLeftEnc = new Encoder(0,0);
		RearRightEnc = new Encoder(0,0);
		EleEnc = new Encoder(0,0);
		
		EleLS = new DigitalInput(0);
		
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
		placeButton.whenPressed(new PlaceCube());
		
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
