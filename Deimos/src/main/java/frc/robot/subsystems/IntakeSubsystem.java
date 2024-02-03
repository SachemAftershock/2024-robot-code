//TODO: Bind intake in, intake out to buttons
package frc.robot.subsystems;


import frc.lib.AftershockSubsystem;
import frc.robot.Constants.IntakeConstants;
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

	private DigitalInput mExternalBeamBreaker;
	private DigitalInput mInternalBeamBreaker;

	private double mIntakeArmMotorSpeed = 0.05;
	private double mIntakeRollerMotorSpeed = 0.05;
	private DigitalInput mIntakeLimitSwitch;
	private boolean isIntakeIn;

	private ProfiledPIDController mIntakeArmPidController;
	private Constraints mIntakeArmPIDConstraints;

	private final Encoder m_encoder = new Encoder(1, 0, false, Encoder.EncodingType.k4X);
  	private ProfiledPIDController encoderPID;

	private IntakeState mDesiredIntakeState;

	private RobotContainer mRobotContainer = RobotContainer.getInstance();

	private IntakeSubsystem() {
		
	}

	@Override
	public void initialize() {
		mIntakeArmMotor = new CANSparkMax(mIntakeArmMotorID, MotorType.kBrushless);
		mIntakeRollerMotor = new CANSparkMax(mIntakeRollerMotorID, MotorType.kBrushless);
		
		mExternalBeamBreaker = new DigitalInput(mExternalBeamBreakerID); //EXTERNAL B1
		mInternalBeamBreaker = new DigitalInput(mInternalBeamBreakerID); //INTERNAL B2

		mIntakeArmEncoder = mIntakeArmMotor.getEncoder();
		mIntakeArmEncoder.setPosition(0);

		mIntakeLimitSwitch = new DigitalInput(0);
		mIntakeArmPIDConstraints = new Constraints(mConstraintsMaxVelocity, mConstraintsMaxAcceleration);
		mIntakeArmPidController = new ProfiledPIDController(mIntakeArmGains[0], mIntakeArmGains[1], mIntakeArmGains[0], mIntakeArmPIDConstraints);

	}

	public void setDesiredState(IntakeState mDesiredIntakeState){
		this.mDesiredIntakeState = mDesiredIntakeState;
		
	}

	//Should be called continuously to keep intake in desiredposition, command returns finished when the PID error is less than a certain epsilon
	public boolean runIntakePID(){
		double mDesiredEncoderValue = mDesiredIntakeState.getPosition();
		double speed = mIntakeArmPidController.calculate(mIntakeArmEncoder.getPosition(), mDesiredEncoderValue);
		mIntakeArmMotor.set(speed);
		// if(mIntakeArmPidController.getPositionError()<.1){//TODO: make espilon
		// 	speed = 0;
		// 	mIntakeArmMotor.set(speed);
		// 	return true;
		// }
		if(mIntakeLimitSwitch.get() && mRobotContainer.getDesiredIntakeState()==IntakeState.eRetracted){
			speed = 0;
			mIntakeArmMotor.set(speed);
			mRobotContainer.setIntakeState(IntakeState.eRetracted);
			return true;
		}
		//TODO: create method to change state based on one or 2 sided epsilon
		return false;
	}

	//Sets the motor idle mode to break or coast, this function may not be as useful as it seems, and only applies extra fiction when current is flowing through the motor, either by movement intentionally or by gravity
	public void lockIntake(){
		mIntakeArmMotor.setIdleMode(IdleMode.kBrake);
	}
	public void coastIntake(){
		mIntakeArmMotor.setIdleMode(IdleMode.kCoast);
	}
	
	public void setRollerMotorSpeed(double speed){
		mIntakeRollerMotor.set(speed); 
	}

	public void ingestNote() {
		double speed = mInjestDonutSpeed;

		if (!mExternalBeamBreaker.get()) {
			if (!mInternalBeamBreaker.get()) {
				speed = 0;
			}
			speed *= 0.5;
		}

		setRollerMotorSpeed(speed);
	}

	public void ejectNote() {
		if (!mExternalBeamBreaker.get() || !mInternalBeamBreaker.get()) {
			setRollerMotorSpeed(mEjectDonutSpeed);
		}
	}
	
	@Override
	public boolean checkSystem() {
		return true;
	}

		// what is telemetry 	@Override //idek breh
	public void outputTelemetry() {

	}

	public synchronized static IntakeSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeSubsystem();
		}
		return mInstance;
	}
}