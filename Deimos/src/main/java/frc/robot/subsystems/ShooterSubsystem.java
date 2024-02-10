//TODO: Set up angle encoder right

package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ShooterConstants.*;
// import frc.robot.enums.ControlState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
	private RelativeEncoder mAngleEncoder;
	private RelativeEncoder mLeftShootEncoder;
	private RelativeEncoder mRightShootEncoder;
	private CANSparkMax mRightShootMotor;
	private DigitalInput mShooterLimitSwitch;
	private DigitalInput mBeamBreakerEnter;
	private DigitalInput mBeamBreakerLeave;

	private boolean mEnableMotors = true;
	// PID for shooting
	private ProfiledPIDController mRightShooterPIDController;
	private Constraints mRightShooterPIDConstraints;
	private ProfiledPIDController mLeftShooterPIDController;
	private Constraints mLeftShooterPIDConstraints;
	private double mShooterConstraintsMaxAcceleration = kShooterConstraintsMaxAcceleration;
	private double[] mShooterGains = kShooterGains;

	// PID for angle stuff
	private double mAngleShootMotorSpeed;
	private ProfiledPIDController mShooterAnglePIDController;
	private TrapezoidProfile.Constraints mShooterAnglePIDConstraints;
	private double[] mAngleGains = kAngleGains;
	double leftSpeed, rightSpeed;

	private ShooterSubsystem() {
		mAngleShootMotor = new CANSparkMax(kAngleShootMotorID, MotorType.kBrushless);
		// mBeamBreakerEnter = new DigitalInput(kBeamBreakerEnterID); // TODO these were temporarily removed
		// mBeamBreakerLeave = new DigitalInput(kBeam`BreakerLeaveID);
		mLeftShootMotor = new CANSparkMax(kLeftShootMotorID, MotorType.kBrushless);
		mRightShootMotor = new CANSparkMax(kRightShootMotorID, MotorType.kBrushless);
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
		mShooterAnglePIDConstraints = new TrapezoidProfile.Constraints(kConstraintsMaxVelocity,
				kAngleMaxAcceleration);
		mShooterAnglePIDController = new ProfiledPIDController(mAngleGains[0], mAngleGains[1], mAngleGains[2],
				mShooterAnglePIDConstraints);
	}

	@Override
	public void initialize() {

	}

	double jogAngle = ShooterAngleState.eSpeaker.getAngle();

	public void manualJogShooter(double speed) {
		mShooterAnglePIDController.calculate(speed);
	}

	/**
	 * The positive direction fires notes upwards
	 * @param leftSpeed [-1.0, 1.0]
	 * @param rightSpeed [-1.0, 1.0]
	 */
	public void spinShooterMotors(double leftSpeed, double rightSpeed) {
		// startRollerMotor method or something here
		System.out.println("Called spin shooter motors");
		// TODO pid not working, stuck at 0
		// leftSpeed = mLeftShooterPIDController.calculate(mLeftShootMotor.getEncoder().getVelocity(), leftSpeed);
		// rightSpeed = mRightShooterPIDController.calculate(mRightShootMotor.getEncoder().getVelocity(), rightSpeed);
		mLeftShootMotor.set(-1.0 * leftSpeed);
		mRightShootMotor.set(rightSpeed); // TODO: USE PID so that speed is consistent despite battery charge/weakness
	}

	/**
	 * Sets angle of the shooter.
	 * 
	 * <p><i>Don't directly set mAngleShootMotor. Route through here instead.</i></p>
	 * @param speed [-1.0, 1.0]. Positive is inward-facing.
	 */
	public void setAngleShootMotorSpeed(double speed) {
		mAngleShootMotor.set(speed);
	}

	private ShooterAngleState mCurrentShooterAngleState;
	
	public void setCurrentShooterAngleState(ShooterAngleState mCurrentShooterAngleState) {
		this.mCurrentShooterAngleState = mCurrentShooterAngleState;
	}
	
	public ShooterAngleState getCurrentShooterAngleState() {
	  return mCurrentShooterAngleState;
	}
	
	private ShooterAngleState mDesiredShooterAngleState;

	public void setDesiredShooterState(ShooterAngleState mDesiredShooterAngleState) {
	  this.mDesiredShooterAngleState = mDesiredShooterAngleState;
	}
  
	public ShooterAngleState getDesiredShooterState() {
	  return mDesiredShooterAngleState;
	}

	// Should be called continuously, returns true when error is less than a certain
	// epsilon
	// public boolean runShooterPID() {
	// 	double mDesiredEncoderValue;
	// 	// if (getControlState() != ControlState.eManualControl) {
	// 		mDesiredEncoderValue = getDesiredShooterState().getAngle();
	// 	// } else {
	// 	// 	mDesiredEncoderValue = jogAngle;
	// 	// }
	// 	double speed = mShooterAnglePIDController.calculate(mAngleEncoder.getPosition(), mDesiredEncoderValue);
	// 	setAngleShootMotorSpeed(speed);
	// 	final double mArmAngleEpsilon = 0.001;
	// 	if (Math.abs(mShooterAnglePIDController.getPositionError()) < mArmAngleEpsilon) {

	// 		mShooterAnglePIDController.setP(kShooterAngleSetPWhenBelowEpsilon); // Test when we have access to robot arm
	// 																			// and which values best
	// 		// counteract gravity
	// 		return true;
	// 	}
	// 	return false;
	// }

	public void runAngleShooterPID(){
		double mDesiredEncoderValue = mDesiredShooterAngleState.getAngle();
		mShooterAnglePIDController.setGoal(mDesiredEncoderValue);

		mAngleShootMotorSpeed = mShooterAnglePIDController.calculate(mAngleEncoder.getPosition(), mDesiredEncoderValue);

		
		System.out.println("-------Desired State encoder: " + mDesiredEncoderValue);
		System.out.println("-------Current State encoder IntakeState: " + mCurrentShooterAngleState.getAngle());
		System.out.println("-------Desired State encoder IntakeState: " + mDesiredShooterAngleState.getAngle());
		System.out.println("CURRENT SPEED: " + mAngleShootMotorSpeed);
		if(Math.abs(mShooterAnglePIDController.getPositionError()) < kShooterAngleEpsilon){//TODO: make espilon
			mAngleShootMotorSpeed = 0;
			if(mEnableMotors) setAngleShootMotorSpeed(mAngleShootMotorSpeed);
			setCurrentShooterAngleState(mDesiredShooterAngleState);
			//return true;
		}
		if(mEnableMotors) setAngleShootMotorSpeed(mAngleShootMotorSpeed);
		// if(Math.abs(mShooterAnglePIDController.getPositionError()) < 0.1){// TODO: add actual epsilon
		// 	setCurrentIntakeState(mDesiredShooterAngleState);
		// }

		// if(mIntakeRetractedLimitSwitch.get()){
		// 	setDesiredIntakeState(IntakeState.eRetracted);
		// 	setCurrentIntakeState(IntakeState.eRetracted);

		// 	mIntakeArmEncoder.setPosition(0.0);
		// } 

		// if(getDesiredIntakeState()==IntakeState.eRetracted && getIntakeState()==IntakeState.eRetracted){
		// 	mSpeed = 0;
		// 	setAngleShootMotorSpeed(mSpeed);
		// 	setCurrentIntakeState(IntakeState.eRetracted);
		// }

	
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
