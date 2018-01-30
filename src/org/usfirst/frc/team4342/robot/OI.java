package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.commands.StartClimber;
import org.usfirst.frc.team4342.robot.commands.DriveGoToAngle;
import org.usfirst.frc.team4342.robot.commands.DriveStraightWithJoystick;
import org.usfirst.frc.team4342.robot.commands.ElevatePickupCube;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleHigh;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleLow;
import org.usfirst.frc.team4342.robot.commands.ElevateToScaleNeutral;
import org.usfirst.frc.team4342.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team4342.robot.commands.StartIntake;
import org.usfirst.frc.team4342.robot.commands.StartRelease;
import org.usfirst.frc.team4342.robot.commands.StopClimber;
import org.usfirst.frc.team4342.robot.commands.StopDrive;
import org.usfirst.frc.team4342.robot.commands.StopElevator;
import org.usfirst.frc.team4342.robot.commands.StopIntake;
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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
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
	public final PowerDistributionPanel PDP;
	
	// Subsystems
	public final Intake Intake;
	public final Elevator Elevator;
	public final TankDrive Drive;
	public final Climber Climber;
	
	// Joysticks, motors, and sensors
	public final Joystick LeftDriveStick, RightDriveStick, SwitchBox;
	public final Talon FrontLeft, FrontRight, RearLeft, RearRight, IntakeMotor;
	public final TalonSRX ClimberMotor, EleMotor;
	public final AHRS NavX;
	public final Encoder LeftDrive, RightDrive, EleEnc;
	public final DigitalInput EleLS;
	
	private OI() {
		Logger.info("Constructing the Operator Interface.....");

		// Power Distribution Panel
		PDP = new PowerDistributionPanel();
		
		// Joysticks and Switch Box
		LeftDriveStick = new Joystick(RobotMap.LEFT_DRIVE_STICK);
		RightDriveStick = new Joystick(RobotMap.RIGHT_DRIVE_STICK);
		SwitchBox = new Joystick(RobotMap.SWITCH_BOX);
		
		// NavX Board
		NavX = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);
		
		// Drive motors
		FrontLeft = new Talon(RobotMap.FRONT_LEFT);
		FrontRight = new Talon(RobotMap.FRONT_RIGHT);
		RearLeft = new Talon(RobotMap.REAR_LEFT);
		RearRight = new Talon(RobotMap.REAR_RIGHT);

		// Climber and Elevator motors
		ClimberMotor = new TalonSRX(RobotMap.CLIMBER_MOTOR);
		EleMotor = new TalonSRX(RobotMap.ELE_MOTOR);

		ClimberMotor.setNeutralMode(NeutralMode.Brake);
		EleMotor.setNeutralMode(NeutralMode.Brake);

		// Intake motor
		IntakeMotor = new Talon(RobotMap.INTAKE_MOTOR);
		
		// Encoders for Drive Train and Elevator
		LeftDrive = new Encoder(RobotMap.LEFT_DRIVE_IN, RobotMap.LEFT_DRIVE_OUT);
		RightDrive = new Encoder(RobotMap.RIGHT_DRIVE_IN, RobotMap.RIGHT_DRIVE_OUT);
		EleEnc = new Encoder(RobotMap.ELE_ENC_IN, RobotMap.ELE_ENC_OUT);
		
		// TODO: Set distance per pulse so encoder values are in inches
		// (Wheel Radius * PI * Num Input Teeth) / (Dist Per Pulse of Encoder * Num Encoder Gear Ratio? * Num Output Teeth)
		LeftDrive.setDistancePerPulse(1);
		RightDrive.setDistancePerPulse(1);
		EleEnc.setDistancePerPulse(1);
		SwitchBox.setTwistChannel(3);
		
		// Limit Switch on the bottom of the elevator
		EleLS = new DigitalInput(RobotMap.ELE_LS);

		// Subsystems
		Intake = new Intake(IntakeMotor);
		Elevator = new Elevator(EleMotor, EleEnc, EleLS);
		Drive = new TankDrive(FrontRight, FrontLeft, RearRight, RearLeft, NavX, LeftDrive, RightDrive);
		Climber = new Climber(ClimberMotor);

		// Climbing button to enable the winch
		// Switch is opposite
		JoystickButton climbButton = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.CLIMB);
		climbButton.whenPressed(new StopClimber(Climber));
		climbButton.whenReleased(new StartClimber(Climber));
		
		// Switch to enable the intake for a cube
		JoystickButton intakeSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.INTAKE);
		intakeSwitch.whenPressed(new StartIntake(Intake));
		intakeSwitch.whenReleased(new StopIntake(Intake));
		
		// Switch to enable reverse intake to release a cube
		JoystickButton releaseSwitch = new JoystickButton(SwitchBox, ButtonMap.SwitchBox.RELEASE);
		releaseSwitch.whenPressed(new StartRelease(Intake));
		releaseSwitch.whenReleased(new StopIntake(Intake));
		
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
		
		// Button on the right drive stick to maintain the robot's current heading
		JoystickButton driveStraight = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_STRAIGHT);
		driveStraight.whenPressed(new DriveStraightWithJoystick(RightDriveStick, Drive));
		driveStraight.whenReleased(new StopDrive(Drive));
		
		// Button on the right drive stick to go to zero heading (facing towards opponent's alliance wall)
		JoystickButton goToZero = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_ZERO);
		goToZero.whenPressed(new DriveGoToAngle(Drive, 0));
		
		// Button on the right drive stick to go to -90 degree heading (facing towards left side of the field)
		JoystickButton goToLeft = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_LEFT);
		goToLeft.whenPressed(new DriveGoToAngle(Drive, -90));
		
		// Button on the right drive stick to go to 90 degree heading (facing towards right side of the field)
		JoystickButton goToRight = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_TO_RIGHT);
		goToRight.whenPressed(new DriveGoToAngle(Drive, 90));
		
		// Button on the right drive stick to go to 180 degree heading (facing towards our alliance wall)
		JoystickButton go180 = new JoystickButton(RightDriveStick, ButtonMap.DriveStick.Right.GO_180);
		go180.whenPressed(new DriveGoToAngle(Drive, 180));
	}
}
