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
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends AftershockSubsystem {

	private static IntakeSubsystem mInstance;
	
	private CANSparkMax mIntakeArmMotor;
	private RelativeEncoder mIntakeArmEncoder;

	private CANSparkMax mIntakeRollerMotor;

	private DigitalInput mExternalBeamBreaker;
	private DigitalInput mInternalBeamBreaker;

	private DigitalInput mIntakeRetractedLimitSwitch;
	
	private ProfiledPIDController mIntakeArmPidController;
	private TrapezoidProfile.Constraints mIntakeArmPIDConstraints;

	//private final Encoder mIntakeArmEncoder;
	private double speed = 0.0;
	//rec  3, trans 4 BEAM BREAKER

	//private IntakeState mDesiredIntakeState;

	private boolean mIntakeCalibrated = false;

	//private RobotContainer mRobotContainer = RobotContainer.getInstance();

	private IntakeSubsystem() {
		
		//mIntakeArmEncoder.setPositionConversionFactor(2000);
		mIntakeArmMotor = new CANSparkMax(kIntakeArmMotorID, MotorType.kBrushless);
		mIntakeArmEncoder = mIntakeArmMotor.getEncoder(); //TODO: make encoder value into constants //TODO: change from DIO to alternate encoder port
		mIntakeArmEncoder.setPosition(0.0);

		mIntakeRollerMotor = new CANSparkMax(kIntakeRollerMotorID, MotorType.kBrushless);

		mExternalBeamBreaker = new DigitalInput(kExternalBeamBreakerID); //EXTERNAL B1
		mInternalBeamBreaker = new DigitalInput(kInternalBeamBreakerID); //INTERNAL B2

		mIntakeRetractedLimitSwitch = new DigitalInput(kIntakeLimitSwitchID);
		mIntakeArmPIDConstraints = new TrapezoidProfile.Constraints(kIntakeConstraintsMaxVelocity, kIntakeConstraintsMaxAcceleration);
		mIntakeArmPidController = new ProfiledPIDController(kIntakeArmGains[0], kIntakeArmGains[1], kIntakeArmGains[2], mIntakeArmPIDConstraints);
		
	}

	@Override
	public void initialize() {
	}

	// public void setDesiredState(IntakeState mDesiredIntakeState){
	// 	this.mDesiredIntakeState = mDesiredIntakeState;
	// }
	private IntakeState mCurrentIntakeState;

	public void setIntakeState(IntakeState CurrentIntakeState) {
		mCurrentIntakeState = CurrentIntakeState;
	  }
	
	  public IntakeState getIntakeState() {
		return mCurrentIntakeState;
	  }
	
	  private IntakeState mDesiredIntakeState;
	
	  public void setDesiredIntakeState(IntakeState DesiredIntakeState) {
		mDesiredIntakeState = DesiredIntakeState;
	  }
	
	  public IntakeState getDesiredIntakeState() {
		return mDesiredIntakeState;
	  }

	public void runIntakePID(){
		if(mIntakeCalibrated){
			runNormalIntakePID();
		} else {
			mIntakeCalibrated = runCalibrateIntake();
		}
	}
	//Should be called continuously to keep intake in desiredposition, command returns finished when the PID error is less than a certain epsilon
	public void runNormalIntakePID(){
		double mDesiredEncoderValue = mDesiredIntakeState.getPosition();
		double speed = mIntakeArmPidController.calculate(mIntakeArmEncoder.getPosition(), mDesiredEncoderValue);
		mIntakeArmMotor.set(speed);
		// if(mIntakeArmPidController.getPositionError()<.1){//TODO: make espilon
		// 	speed = 0;
		// 	mIntakeArmMotor.set(speed);
		// 	return true;
		// }
		if(Math.abs(mIntakeArmPidController.getPositionError())<.1){// TODO: add actual epsilon
			setIntakeState(mDesiredIntakeState);
		}
		if(mIntakeRetractedLimitSwitch.get()){
			setDesiredIntakeState(IntakeState.eRetracted);
		}
		if(getDesiredIntakeState()==IntakeState.eRetracted && getIntakeState()==IntakeState.eRetracted){
			speed = 0;
			mIntakeArmMotor.set(speed);
			setIntakeState(IntakeState.eRetracted);
		}
		//TODO: create method to change state based on one or 2 sided epsilon
	
	}
	public boolean runCalibrateIntake(){
		final double calibrationRetractionSpeed = 0.05;  // Percent
		if(!mIntakeRetractedLimitSwitch.get()){
			mIntakeArmMotor.set(-calibrationRetractionSpeed);
		}else{
			double speed = 0;
			mIntakeArmMotor.set(speed);
			setDesiredIntakeState(IntakeState.eRetracted);
			setIntakeState(IntakeState.eRetracted);
			return true;
		}	
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
	
	public void setIntakeArmMotorSpeed() {
		if(speed < 1.0){
			speed += 0.1;	
		}
		else
		{
			speed = 1.0;
		}
		mIntakeArmMotor.set(speed);
		System.out.println("INTAKE ARM MOTOR SPEED: " + speed + "-------------");
		//System.out.println("ENCODER VALUES: " + mIntakeArmEncoder.getPosition() + "-------");

	}
	//TEMP
	public void setNegIntakeArmMotorSpeed() {
		if(speed > -1.0){
			speed -= 0.1;	
		}
		else
		{
			speed = -1.0;
		}
		mIntakeArmMotor.set(speed);
		System.out.println("INTAKE ARM MOTOR SPEED: " + speed + "-------------");
		//System.out.println("ENCODER VALUES: " + mIntakeArmEncoder.getPosition() + "-------");

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
			setIntakeState(IntakeState.eRetracted);
			//mIntakeArmEncoder.setPosition(0.0);//   .reset();
		}
		System.out.println("ENCODER VALUES: " + mIntakeArmEncoder.getPosition() + "-------");



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