//TODO: Set up angleencoder right

package frc.robot.subsystems;


import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
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
	private RelativeEncoder mAngleEncoder; 
	private RelativeEncoder mLeftShootEncoder;
	private RelativeEncoder mRightShootEncoder;
	private CANSparkMax mRightShootMotor; 
	private DigitalInput mShooterLimitSwitch;
	private DigitalInput mBeamBreakerEnter; 
	private DigitalInput mBeamBreakerLeave;

	private RobotContainer mRobotContainer = RobotContainer.getInstance();
	//PID for shooting
	private ProfiledPIDController mRightShooterPIDController;
	private Constraints mRightShooterPIDConstraints;
	private ProfiledPIDController mLeftShooterPIDController;
	private Constraints mLeftShooterPIDConstraints;
	private ProfiledPIDController mShooterAnglePIDController;
	private Constraints mShooterAnglePIDConstraints;
	private double mShooterConstraintsMaxAcceleration = ShooterConstants.mShooterConstraintsMaxAcceleration;
	private double[] mShooterGains = ShooterConstants.mShooterGains;


	//PID for angle stuff
	private ProfiledPIDController mAnglePidController;
	private Constraints mAnglePIDConstraints;
	private double[] mAngleGains= ShooterConstants.mAngleGains;
	double leftSpeed, rightSpeed;
	
	private ShooterSubsystem() {
		mBeamBreakerEnter = new DigitalInput(ShooterConstants.mBeamBreakerEnterID);
		mBeamBreakerLeave = new DigitalInput(ShooterConstants.mBeamBreakerLeaveID);
		mLeftShootMotor = new CANSparkMax(ShooterConstants.mLeftShootMotorID, MotorType.kBrushless);
		mRightShootMotor = new CANSparkMax(ShooterConstants.mRightShootMotorID, MotorType.kBrushless);
		mLeftShootEncoder = mLeftShootMotor.getEncoder();
		mRightShootEncoder = mRightShootMotor.getEncoder();
		mAngleEncoder = mAngleShootMotor.getAlternateEncoder(5);
		mLeftShootEncoder.setPosition(0);
		mRightShootEncoder.setPosition(0);
		mShooterLimitSwitch = new DigitalInput(0);
		mLeftShooterPIDConstraints = new Constraints(ShooterConstants.mConstraintsMaxVelocity, mShooterConstraintsMaxAcceleration);
		mLeftShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2], mLeftShooterPIDConstraints);
		mRightShooterPIDConstraints = new Constraints(ShooterConstants.mConstraintsMaxVelocity, mShooterConstraintsMaxAcceleration);
		mRightShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2], mRightShooterPIDConstraints);
		mShooterAnglePIDConstraints = new Constraints(ShooterConstants.mConstraintsMaxVelocity, ShooterConstants.mAngleConstraintsMaxAcceleration);
		mShooterAnglePIDController = new ProfiledPIDController(mAngleGains[0], mAngleGains[1], mAngleGains[2], mAnglePIDConstraints);
	}

	@Override
	public void initialize() {

	}
	double jogAngle = ShooterState.eSpeaker.getAngle();
	
	public void manualJogShooter(double speed){
		mAnglePidController.set(speed);
	}
	


	public void spinShooterMotors(double leftSpeed, double rightSpeed){
		// startRollerMotor method or something here
		

		leftSpeed = mLeftShooterPIDController.calculate(mLeftShootMotor.getEncoder().getVelocity(), leftSpeed);
		rightSpeed = mRightShooterPIDController.calculate(mRightShootMotor.getEncoder().getVelocity(), rightSpeed);
		mLeftShootMotor.set(leftSpeed);
		mRightShootMotor.set(rightSpeed); //TODO: USE PID so that speed is consistent despite battery charge/weakness
	}
	
	//Should be called continuously, returns true when error is less than a certain epsilon
	public boolean runShooterPID(){
		double mDesiredEncoderValue;
	if(mRobotContainer.getControlState()!=ControlState.eManualControl){
		mDesiredEncoderValue = mRobotContainer.getDesiredShooterState().getAngle();
	}else{
		mDesiredEncoderValue = jogAngle;
	}
	double speed = mShooterAnglePIDController.calculate(mAngleEncoder.getPosition(), mDesiredEncoderValue);
	mAngleShootMotor.set(speed);
	final double mArmAngleEpsilon = 0.001;
	if(Math.abs(mAnglePidController.getPositionError()) < mArmAngleEpsilon){

	mShooterAnglePIDController.setP(4); //Test when we have access to robot arm and which values best counteract gravity
		return true;
	}
	return false;
}

	@Override
	public void outputTelemetry(){
		ShuffleboardTab twb = Shuffleboard.getTab("SubsystemShooter");
		twb.add("E",false).getEntry();
		//TODO:Dump current encoder speeds
	}

	public synchronized static ShooterSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ShooterSubsystem();
		}
		return mInstance;
		
	}
}

