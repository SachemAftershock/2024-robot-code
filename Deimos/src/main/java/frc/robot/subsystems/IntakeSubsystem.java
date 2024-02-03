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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends AftershockSubsystem {

	private static IntakeSubsystem mInstance;
	
	private CANSparkMax mIntakeArmMotor;
	//private RelativeEncoder mIntakeArmEncoder;
	private CANSparkMax mIntakeRollerMotor;

	private DigitalInput mExternalBeamBreaker;
	private DigitalInput mInternalBeamBreaker;

	private DigitalInput mIntakeRetractedLimitSwitch;
	
	private ProfiledPIDController mIntakeArmPidController;
	private TrapezoidProfile.Constraints mIntakeArmPIDConstraints;

	private final Encoder mIntakeArmEncoder;

	private IntakeState mDesiredIntakeState;

	private RobotContainer mRobotContainer = RobotContainer.getInstance();

	private IntakeSubsystem() {
		mIntakeArmEncoder = new Encoder(1, 0, false, Encoder.EncodingType.k4X); //TODO: make encoder value into constants //TODO: change from DIO to alternate encoder port
		mIntakeArmMotor = new CANSparkMax(kIntakeArmMotorID, MotorType.kBrushless);
		mIntakeRollerMotor = new CANSparkMax(kIntakeRollerMotorID, MotorType.kBrushless);
		mExternalBeamBreaker = new DigitalInput(kExternalBeamBreakerID); //EXTERNAL B1
		mInternalBeamBreaker = new DigitalInput(kInternalBeamBreakerID); //INTERNAL B2
		mIntakeRetractedLimitSwitch = new DigitalInput(kIntakeLimitSwitchID);
		mIntakeArmPIDConstraints = new TrapezoidProfile.Constraints(kIntakeConstraintsMaxVelocity, kIntakeConstraintsMaxAcceleration);
		mIntakeArmPidController = new ProfiledPIDController(kIntakeArmGains[0], kIntakeArmGains[1], kIntakeArmGains[0], mIntakeArmPIDConstraints);

	}

	@Override
	public void initialize() {


		
	}

	public void setDesiredState(IntakeState mDesiredIntakeState){
		this.mDesiredIntakeState = mDesiredIntakeState;
		
	}

	//Should be called continuously to keep intake in desiredposition, command returns finished when the PID error is less than a certain epsilon
	public void runIntakePID(){
		double mDesiredEncoderValue = mDesiredIntakeState.getPosition();
		double speed = mIntakeArmPidController.calculate(mIntakeArmEncoder.getDistance(), mDesiredEncoderValue);
		mIntakeArmMotor.set(speed);
		// if(mIntakeArmPidController.getPositionError()<.1){//TODO: make espilon
		// 	speed = 0;
		// 	mIntakeArmMotor.set(speed);
		// 	return true;
		// }
		if(Math.abs(mIntakeArmPidController.getPositionError())<.1){// TODO: add actual epsilon
			mRobotContainer.setIntakeState(mDesiredIntakeState);
		}
		if(mIntakeRetractedLimitSwitch.get()){
			mRobotContainer.setDesiredIntakeState(IntakeState.eRetracted);
		}
		if(mRobotContainer.getDesiredIntakeState()==IntakeState.eRetracted && mRobotContainer.getIntakeState()==IntakeState.eRetracted){
			speed = 0;
			mIntakeArmMotor.set(speed);
			mRobotContainer.setIntakeState(IntakeState.eRetracted);
		}
		//TODO: create method to change state based on one or 2 sided epsilon
	
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

	public void setIntakeArmMotorSpeed(double speed) {
		mIntakeArmMotor.set(speed);
	}

	public void ingestNote() {
		double speed = kIngestNoteSpeed;

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
			setRollerMotorSpeed(kEjectNoteSpeed);
		}
	}
	
	@Override
	public boolean checkSystem() {
		return true;
	}

	@Override
	public void periodic(){
		//call statecheck method, ... make statecheck call
		if(mIntakeRetractedLimitSwitch.get()){
			mRobotContainer.setIntakeState(IntakeState.eRetracted);
			mIntakeArmEncoder.reset();
		}
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