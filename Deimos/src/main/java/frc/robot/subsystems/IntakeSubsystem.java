//TODO: Bind intake in, intake out to buttons
package frc.robot.subsystems;


import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends AftershockSubsystem {

	private static IntakeSubsystem mInstance;
	
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
	private IntakeState mDesiredIntakeState;
	private RobotContainer mRobotContainer = RobotContainer.getInstance();

	private IntakeSubsystem() {
		mIntakeArmMotor = new CANSparkMax(mIntakeArmMotorID, MotorType.kBrushless);
		mIntakeRollerMotor = new CANSparkMax(mIntakeRollerMotorID, MotorType.kBrushless);
		mIntakeArmEncoder = mIntakeArmMotor.getEncoder();
		mIntakeArmEncoder.setPosition(0);
		mIntakeLimitSwitch = new DigitalInput(0);
		mIntakeArmPIDConstraints = new Constraints(mConstraintsMaxVelocity, mConstraintsMaxAcceleration);
		mIntakeArmPidController = new ProfiledPIDController(mIntakeArmGains[0], mIntakeArmGains[1], mIntakeArmGains[0], mIntakeArmPIDConstraints);
	}

	@Override
	public void initialize() {

	}

	public void setDesiredState(IntakeState mDesiredIntakeState){
		this.mDesiredIntakeState = mDesiredIntakeState;
		
	}

	public boolean runIntakePID(){
		double mDesiredEncoderValue = mDesiredIntakeState.getPosition();
		double speed = mIntakeArmPidController.calculate(mIntakeArmEncoder.getPosition(), mDesiredEncoderValue);
		mIntakeArmMotor.set(speed);
		if (mIntakeArmPidController.getPositionError() < kArmPIDPositionErrorEpsilon) {  //TODO: make espilon
			speed = 0;
			mIntakeArmMotor.set(speed);
			return true;
		}
		return false;
	}

	
	public void lockIntake(){
		mIntakeArmMotor.setIdleMode(IdleMode.kBrake);
	}
	
	public void coastIntake(){
		mIntakeArmMotor.setIdleMode(IdleMode.kCoast);
	}
	
	public void setRollerMotorSpeed(double speed){
		mIntakeRollerMotor.set(speed);
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

	public synchronized static IntakeSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeSubsystem();
		}
		return mInstance;
	}
}
