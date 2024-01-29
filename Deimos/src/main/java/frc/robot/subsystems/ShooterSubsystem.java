//TODO: Set up angleencoder right

package frc.robot.subsystems;


import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends AftershockSubsystem {

	private static ShooterSubsystem mInstance;
	
	private CANSparkMax mAngleShootMotor;
	private double mShooterEncoderSetPoint = 0.5;
	private CANSparkMax mLeftShootMotor;
	private RelativeEncoder mAngleEncoder;//TODO: Implement actual encoders anywhere where there will be encoders
	private int mAngleEncoderID1 = 0;
	private int mAngleEncoderID2 = 1;
	private RelativeEncoder mLeftShootEncoder;
	private RelativeEncoder mRightShootEncoder;
	private CANSparkMax mRightShootMotor;
	private double mLeftShootMotorSpeed = 0.05;
	private double mRightShootMotorSpeed = 0.05;
	private DigitalInput mShooterLimitSwitch;
	private int mLeftShootMotorID = 0;
	private int mRightShootMotorID = 1;
	private int mAngleShootMotorID = 2;

	private RobotContainer mRobotContainer = RobotContainer.getInstance();
	//PID for shooting
	private ProfiledPIDController mRightShooterPIDController;
	private Constraints mRightShooterPIDConstraints;
	private ProfiledPIDController mLeftShooterPIDController;
	private Constraints mLeftShooterPIDConstraints;
	private ProfiledPIDController mShooterAnglePIDController;
	private Constraints mShooterAnglePIDConstraints;
	private double mConstraintsMaxVelocity = 0;
	private double mShooterConstraintsMaxAcceleration = 0;
	private double[] mShooterGains= {0.4,0,0};


	//PID for angle stuff
	private ProfiledPIDController mAnglePidController;
	private Constraints mAnglePIDConstraints;
	private double mAngleMaxVelocity = 0;
	private double mAngleConstraintsMaxAcceleration = 0;
	private double[] mAngleGains= {0.4,0,0};
	double leftSpeed, rightSpeed;
	private ShooterSubsystem() {
		mLeftShootMotor = new CANSparkMax(mLeftShootMotorID, MotorType.kBrushless);
		mRightShootMotor = new CANSparkMax(mRightShootMotorID, MotorType.kBrushless);
		mLeftShootEncoder = mLeftShootMotor.getEncoder();
		mRightShootEncoder = mRightShootMotor.getEncoder();
		mAngleEncoder = mAngleShootMotor.getAlternateEncoder(5);//TODO - define encoder as seperate object
		mLeftShootEncoder.setPosition(0);
		mRightShootEncoder.setPosition(0);
		mShooterLimitSwitch = new DigitalInput(0);
		mLeftShooterPIDConstraints = new Constraints(mConstraintsMaxVelocity, mShooterConstraintsMaxAcceleration);
		mLeftShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2], mLeftShooterPIDConstraints);
		mRightShooterPIDConstraints = new Constraints(mConstraintsMaxVelocity, mShooterConstraintsMaxAcceleration);
		mRightShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2], mRightShooterPIDConstraints);
		mShooterAnglePIDConstraints = new Constraints(mConstraintsMaxVelocity, mShooterConstraintsMaxAcceleration);
		mShooterAnglePIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2], mShooterAnglePIDConstraints);
	}

	@Override
	public void initialize() {

	}

	public void spinShooterMotors(double leftSpeed, double rightSpeed){
		// startRollerMotor method or something here
		//leftSpeed = mLeftShooterPIDController.calculate(mLeftShootMotor.getEncoder().getVelocity(), leftSpeed);
		//rightSpeed = mRightShooterPIDController.calculate(mRightShootMotor.getEncoder().getVelocity(), rightSpeed);
		mLeftShootMotor.set(leftSpeed);
		mRightShootMotor.set(rightSpeed);
	}

	//ShooterState mDesiredShooterState;
	// public void setDesiredState(ShooterState mDesiredShooterState){
	// 	this.mDesiredShooterState = mDesiredShooterState;
		
	// }
	public boolean runShooterPID(){
		double mDesiredEncoderValue = mRobotContainer.getDesiredShooterState().getAngle();
		double speed = mShooterAnglePIDController.calculate(mAngleEncoder.getPosition(), mDesiredEncoderValue);
		mAngleShootMotor.set(speed);
		if(mAnglePidController.getPositionError()<.1){//TODO: make espilon
			speed = 0;
			mAngleShootMotor.set(speed);
			return true;
		}
		return false;
	}

	@Override
	public void outputTelemetry(){

	}

	public synchronized static ShooterSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ShooterSubsystem();
		}
		return mInstance;
	}
}

