//TODO: Set up angle encoder right

package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.enums.ControlState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends AftershockSubsystem {

	private static ShooterSubsystem mInstance;

	private CANSparkMax mAngleShootMotor;
	private CANSparkMax mLeftShootMotor;
	private CANSparkMax mShooterArmMotor;
	private RelativeEncoder mAngleEncoder;
	private RelativeEncoder mLeftShootEncoder;
	private RelativeEncoder mRightShootEncoder;
	private CANSparkMax mRightShootMotor;
	private DigitalInput mShooterLimitSwitch;
	private DigitalInput mBeamBreakerEnter;
	private DigitalInput mBeamBreakerLeave;

	private RobotContainer mRobotContainer = RobotContainer.getInstance();
	// PID for shooting
	private ProfiledPIDController mRightShooterPIDController;
	private Constraints mRightShooterPIDConstraints;
	private ProfiledPIDController mLeftShooterPIDController;
	private Constraints mLeftShooterPIDConstraints;
	private ProfiledPIDController mShooterAnglePIDController;
	private Constraints mShooterAnglePIDConstraints;
	private double mShooterConstraintsMaxAcceleration = kShooterConstraintsMaxAcceleration;
	private double[] mShooterGains = kShooterGains;

	// PID for angle stuff
	private ProfiledPIDController mAnglePidController;
	private Constraints mAnglePIDConstraints;
	private double[] mAngleGains = kAngleGains;
	double leftSpeed, rightSpeed;

	private ShooterSubsystem() {
		mBeamBreakerEnter = new DigitalInput(kBeamBreakerEnterID);
		mBeamBreakerLeave = new DigitalInput(kBeamBreakerLeaveID);
		mLeftShootMotor = new CANSparkMax(kLeftShootMotorID, MotorType.kBrushless);
		mRightShootMotor = new CANSparkMax(kRightShootMotorID, MotorType.kBrushless);
		mShooterArmMotor = new CANSparkMax(kShooterArmMotorID, MotorType.kBrushless);
		mLeftShootEncoder = mLeftShootMotor.getEncoder();
		mRightShootEncoder = mRightShootMotor.getEncoder();
		mAngleEncoder = mAngleShootMotor.getAlternateEncoder(5);
		mLeftShootEncoder.setPosition(0);
		mRightShootEncoder.setPosition(0);
		mShooterLimitSwitch = new DigitalInput(kShooterLimitSwitchID);
		mLeftShooterPIDConstraints = new Constraints(kConstraintsMaxVelocity,
				mShooterConstraintsMaxAcceleration);
		mLeftShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2],
				mLeftShooterPIDConstraints);
		mRightShooterPIDConstraints = new Constraints(kConstraintsMaxVelocity,
				mShooterConstraintsMaxAcceleration);
		mRightShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2],
				mRightShooterPIDConstraints);
		mShooterAnglePIDConstraints = new Constraints(kConstraintsMaxVelocity,
				kAngleMaxAcceleration);
		mShooterAnglePIDController = new ProfiledPIDController(mAngleGains[0], mAngleGains[1], mAngleGains[2],
				mAnglePIDConstraints);
	}

	@Override
	public void initialize() {

	}

	double jogAngle = ShooterState.eSpeaker.getAngle();

	public void manualJogShooter(double speed) {
		mAnglePidController.calculate(speed);
	}

	public void spinShooterMotors(double leftSpeed, double rightSpeed) {
		// startRollerMotor method or something here

		leftSpeed = mLeftShooterPIDController.calculate(mLeftShootMotor.getEncoder().getVelocity(), leftSpeed);
		rightSpeed = mRightShooterPIDController.calculate(mRightShootMotor.getEncoder().getVelocity(), rightSpeed);
		mLeftShootMotor.set(leftSpeed);
		mRightShootMotor.set(rightSpeed); // TODO: USE PID so that speed is consistent despite battery charge/weakness
	}

	public void setShooterArmMotorSpeed(double speed) {
		mShooterArmMotor.set(speed);
	}


	// Should be called continuously, returns true when error is less than a certain
	// epsilon
	public boolean runShooterPID() {
		double mDesiredEncoderValue;
		if (mRobotContainer.getControlState() != ControlState.eManualControl) {
			mDesiredEncoderValue = mRobotContainer.getDesiredShooterState().getAngle();
		} else {
			mDesiredEncoderValue = jogAngle;
		}
		double speed = mShooterAnglePIDController.calculate(mAngleEncoder.getPosition(), mDesiredEncoderValue);
		mAngleShootMotor.set(speed);
		final double mArmAngleEpsilon = 0.001;
		if (Math.abs(mAnglePidController.getPositionError()) < mArmAngleEpsilon) {

			mShooterAnglePIDController.setP(kShooterAngleSetPWhenBelowEpsilon); // Test when we have access to robot arm
																				// and which values best
			// counteract gravity
			return true;
		}
		return false;
	}

	public ProfiledPIDController getShooterAnglePIDController() {
		return mShooterAnglePIDController;
	}

	@Override
	public void outputTelemetry() {
		ShuffleboardTab tab = Shuffleboard.getTab("SubsystemShooter");
		tab.add("Angle Encoder: ", mAngleEncoder.getVelocity()).getEntry();
		tab.add("Left Shooter Encoder: ", mLeftShootEncoder.getVelocity()).getEntry();
		tab.add("Right Shooter Encoder: ", mRightShootEncoder.getVelocity()).getEntry();
		// TODO:Dump current encoder speeds
		// private RelativeEncoder mAngleEncoder;
		// private RelativeEncoder mLeftShootEncoder;
		// private RelativeEncoder mRightShootEncoder;
	}

	public synchronized static ShooterSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ShooterSubsystem();
		}
		return mInstance;

	}
}
