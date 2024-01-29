//dont know point of this yet

package frc.robot.subsystems;


import frc.lib.AftershockSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class SuperstructureSubsystem extends AftershockSubsystem {

	private static SuperstructureSubsystem mInstance;
	
	private double mIntakeArmEncoderSetPoint = 0.5;
	private CANSparkMax mIntakeArmMotor;
	private RelativeEncoder mIntakeArmEncoder;
	private CANSparkMax mIntakeRollerMotor;
	private double mIntakeArmMotorSpeed = 0.05;
	private double mIntakeRollerMotorSpeed = 0.05;
	private DigitalInput mIntakeLimitSwitch;
	private boolean isIntakeIn;
	private int mIntakeArmMotorID = 0;
	private int mIntakeRollerMotorID = 1;

	private ProfiledPIDController mIntakeArmPidController;
	private Constraints mIntakeArmPIDConstraints;
	private double mConstraintsMaxVelocity = 0;
	private double mConstraintsMaxAcceleration = 0;
	private double[] mIntakeArmGains= {0.4,0,0};
	private final Encoder m_encoder = new Encoder(1, 0, false, Encoder.EncodingType.k4X);
  	private ProfiledPIDController encoderPID;
	private SuperstructureSubsystem() {
		mIntakeArmMotor = new CANSparkMax(mIntakeArmMotorID, MotorType.kBrushless);
		mIntakeRollerMotor = new CANSparkMax(mIntakeRollerMotorID, MotorType.kBrushless);
		mIntakeArmEncoder = mIntakeArmMotor.getEncoder();
		mIntakeArmEncoder.setPosition(0);
		mIntakeLimitSwitch = new DigitalInput(0);
		isIntakeIn = true;
		mIntakeArmPIDConstraints = new Constraints(mConstraintsMaxVelocity, mConstraintsMaxAcceleration);
		mIntakeArmPidController = new ProfiledPIDController(mIntakeArmGains[0], mIntakeArmGains[1], mIntakeArmGains[0], mIntakeArmPIDConstraints);
	}

	@Override
	public void initialize() {

	}

	public void move(double distance){
		double speed = mIntakeArmPidController.calculate(mIntakeArmEncoder.getPosition(), distance);

	}

	//Will eventually be a command - Moves intake out and starts rollers
	public void intakeOut(){
		if(mIntakeArmEncoder.getPosition()<mIntakeArmEncoderSetPoint){		//stop motor after certain encoder number is reached
			mIntakeArmMotor.set(mIntakeArmMotorSpeed);//move intake motor out
		}else{
			mIntakeArmMotor.set(0);
		}
		isIntakeIn = false;
		lockIntake();
		startRollerMotors();
		
	}

	//Will eventually be a command - moves intake in and stop rollers
	public void intakeIn(){

	}

	//currently unused, to be used by outside functions
	public boolean checkIsIntakeIn(){
		
		
		
		return isIntakeIn;
	}

	public void lockIntake(){
		mIntakeArmMotor.setIdleMode(IdleMode.kBrake);
	}
	
	public void coastIntake(){
		mIntakeArmMotor.setIdleMode(IdleMode.kCoast);
	}
	
	public void startRollerMotors(){
		mIntakeRollerMotor.set(mIntakeRollerMotorSpeed);
	}
	
	public void stopRollerMotors(){
		mIntakeRollerMotor.set(0);
	}


//why do we have a periodic method if we are looping these checks as a command	@Override
	public void periodic() {

	}

	
	@Override
	public boolean checkSystem() {
		return true;
	}

		// what is telemetry 	@Override
	public void outputTelemetry() {

	}

	public synchronized static SuperstructureSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new SuperstructureSubsystem();
		}
		return mInstance;
	}
}
