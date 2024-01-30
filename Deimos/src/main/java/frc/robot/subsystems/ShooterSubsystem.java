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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends AftershockSubsystem {

	private static ShooterSubsystem mInstance;
	
	private CANSparkMax mAngleShootMotor;
	private double mShooterEncoderSetPoint = 0.5; //Derive empirically
	private CANSparkMax mLeftShootMotor;
	private Encoder mAngleEncoder;//TODO: Implement actual encoders anywhere where there will be encoders
	private int mAngleEncoderID1 = 0; //TODO: move to constants
	private int mAngleEncoderID2 = 1;
	private RelativeEncoder mLeftShootEncoder;
	private RelativeEncoder mRightShootEncoder;
	private CANSparkMax mRightShootMotor;
	private double mLeftShootMotorSpeed = 0.05; //Derive empirically 
	private double mRightShootMotorSpeed = 0.05; //Derive empirically 
	private DigitalInput mShooterLimitSwitch;
	private int mLeftShootMotorID = 0;
	private int mRightShootMotorID = 1; //TODO:Move to constants
	private int mAngleShootMotorID = 2;

	private RobotContainer mRobotContainer = RobotContainer.getInstance();
	//PID for shooting
	private ProfiledPIDController mRightShooterPIDController;
	private Constraints mRightShooterPIDConstraints;
	private ProfiledPIDController mLeftShooterPIDController;
	private Constraints mLeftShooterPIDConstraints;
	private ProfiledPIDController mShooterAnglePIDController;
	private Constraints mShooterAnglePIDConstraints;
	private double mConstraintsMaxVelocity = .5;
	private double mShooterConstraintsMaxAcceleration = .5;
	private double[] mShooterGains= {0.4,0,0};


	//PID for angle stuff
	private ProfiledPIDController mAnglePidController;
	private Constraints mAnglePIDConstraints;
	private double mAngleMaxVelocity = 0.5;
	private double mAngleConstraintsMaxAcceleration = 0.5;
	private double[] mAngleGains= {0.4,0,0};
	double leftSpeed, rightSpeed;
	
	private ShooterSubsystem() {
		mLeftShootMotor = new CANSparkMax(mLeftShootMotorID, MotorType.kBrushless);
		mRightShootMotor = new CANSparkMax(mRightShootMotorID, MotorType.kBrushless);
		mLeftShootEncoder = mLeftShootMotor.getEncoder();
		mRightShootEncoder = mRightShootMotor.getEncoder();
		mAngleEncoder = mAngleShootMotor.getAlternateEncoder(5);//TODO - define encoder as seperate object
		mLeftShootEncoder.setPosition(0);  //TODO: make it a reset encoder count to 0
		mRightShootEncoder.setPosition(0);
		mShooterLimitSwitch = new DigitalInput(0);
		mLeftShooterPIDConstraints = new Constraints(mConstraintsMaxVelocity, mShooterConstraintsMaxAcceleration);
		mLeftShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2], mLeftShooterPIDConstraints);
		mRightShooterPIDConstraints = new Constraints(mConstraintsMaxVelocity, mShooterConstraintsMaxAcceleration);
		mRightShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2], mRightShooterPIDConstraints);
		mShooterAnglePIDConstraints = new Constraints(mConstraintsMaxVelocity, mAngleConstraintsMaxAcceleration);
		mShooterAnglePIDController = new ProfiledPIDController(mAngleGains[0], mAngleGains[1], mAngleGains[2], mAnglePIDConstraints);
	}

	@Override
	public void initialize() {

	}

	public void spinShooterMotors(double leftSpeed, double rightSpeed){
		// startRollerMotor method or something here
		//leftSpeed = mLeftShooterPIDController.calculate(mLeftShootMotor.getEncoder().getVelocity(), leftSpeed);
		//rightSpeed = mRightShooterPIDController.calculate(mRightShootMotor.getEncoder().getVelocity(), rightSpeed);
		mLeftShootMotor.set(leftSpeed);
		mRightShootMotor.set(rightSpeed); //TODO: USE PID so that speed is consistent despite battery charge/weakness
	}
	
	//ShooterState mDesiredShooterState;
	// public void setDesiredState(ShooterState mDesiredShooterState){
	// 	this.mDesiredShooterState = mDesiredShooterState;
		
	// }
	public boolean runShooterPID(){
		double mDesiredEncoderValue = mRobotContainer.getDesiredShooterState().getAngle();
		double speed = mShooterAnglePIDController.calculate(mAngleEncoder.getPosition(), mDesiredEncoderValue);
		mAngleShootMotor.set(speed);
		if(mAnglePidController.getPositionError()<.1){//TODO: make epsilon
			speed = 0; //TODO:Delete, dont change speed just set flag
			mAngleShootMotor.set(speed);
			return true;
		}
		return false;
	}
													
	@Override
	public void outputTelemetry(){
		//ShuffleboardTab twb = new Shuffleboard.getTab("Subsystemshooter")
		//.add("E",false);
		//TODO:Dump current encoder speeds
	}

	public synchronized static ShooterSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ShooterSubsystem();
		}
		return mInstance;
	}
}

