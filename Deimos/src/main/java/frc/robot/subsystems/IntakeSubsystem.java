package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends AftershockSubsystem {

	private static IntakeSubsystem mInstance;
	
	private CANSparkMax mIntakeArmMotor;
	private RelativeEncoder mIntakeArmEncoder;

	private CANSparkMax mIntakeRollerMotor;

	private DigitalInput mExternalBeamBreaker;
	private DigitalInput mInternalBeamBreaker;
	private DigitalInput mIntakeRetractedLimitSwitch;
	
	public enum  IntakeArmPositionEnum { eUnknown, eDeployed, eRetracted };
	private IntakeArmPositionEnum mDesiredIntakeArmPosition = IntakeArmPositionEnum.eUnknown;

	final private double kDesiredIntakeArmEncoderSweep = 7.4; //8.0;

	final private boolean kEnableMotors = true;

	private IntakeSubsystem() {
		
		//mIntakeArmEncoder.setPositionConversionFactor(2000);
		mIntakeArmMotor = new CANSparkMax(kIntakeArmMotorID, MotorType.kBrushless);
		mIntakeArmEncoder = mIntakeArmMotor.getEncoder(); 
		
		mIntakeRollerMotor = new CANSparkMax(kIntakeRollerMotorID, MotorType.kBrushless);

		mExternalBeamBreaker = new DigitalInput(kExternalBeamBreakerID); //EXTERNAL B1
		mInternalBeamBreaker = new DigitalInput(kInternalBeamBreakerID); //INTERNAL B2

		mIntakeRetractedLimitSwitch = new DigitalInput(kIntakeLimitSwitchID);
	}
	
	@Override
	public void initialize() {
		mIntakeArmMotor.setIdleMode(IdleMode.kBrake);
		calibrateArm();
	}

	public void calibrateArm() {
		if(mIntakeRetractedLimitSwitch.get()) {
			mIntakeArmEncoder.setPosition(0.0);
		} else {
			mIntakeArmEncoder.setPosition(-kDesiredIntakeArmEncoderSweep);
		}
	}

	public void RetractIntake() {
		mDesiredIntakeArmPosition = IntakeArmPositionEnum.eRetracted;
	}

	public void DeployIntake() {
		mDesiredIntakeArmPosition = IntakeArmPositionEnum.eDeployed;
	}

	public IntakeArmPositionEnum getIntakeArmState() {
		double epsilon = 0.1;
		IntakeArmPositionEnum currentIntakeArmPosition;
		if(mIntakeRetractedLimitSwitch.get())
			currentIntakeArmPosition = IntakeArmPositionEnum.eRetracted;
		else if ((Math.abs(mIntakeArmEncoder.getPosition()) < epsilon) 
			  || (Math.abs(mIntakeArmEncoder.getPosition()) > kDesiredIntakeArmEncoderSweep - epsilon)) 
			currentIntakeArmPosition = IntakeArmPositionEnum.eDeployed;
		else
			currentIntakeArmPosition = IntakeArmPositionEnum.eUnknown;
		return currentIntakeArmPosition;
	}

	
	public void runControlIntakeArmPosition() {
		final boolean showPrints = false;		
		double mMaximumIntakeArmUpswingLiftMaxSpeed = 0;
		double mMaximumIntakeArmDownswingBrakingMaxSpeed = 0;
		double EncoderCountThresholdToReverseDirection = kDesiredIntakeArmEncoderSweep / 2.0;
		double currentIntakeArmEncoderPosition = mIntakeArmEncoder.getPosition();
		double intakeArmSpeed = 0;
		double factor = 0;

		if (mDesiredIntakeArmPosition == IntakeArmPositionEnum.eRetracted) {

			if (getIntakeArmState() == IntakeArmPositionEnum.eRetracted) {
				intakeArmSpeed = 0.1;  // Apply persisting parking pressure, to counter robot motion dynamics
				if (showPrints) System.out.print("Phase R3: ");
			} else {
				mMaximumIntakeArmUpswingLiftMaxSpeed = 0.6;
				mMaximumIntakeArmDownswingBrakingMaxSpeed = -0.03;
				EncoderCountThresholdToReverseDirection = 2.5;

				if (Math.abs(currentIntakeArmEncoderPosition) > EncoderCountThresholdToReverseDirection) {
					// from deployed position, start with maximum lift speed but then ramp it down propotionaly to zero
					// along to apogee position.   Still a bit of momentum towards retracted position when at apogee.
					factor = (Math.abs(currentIntakeArmEncoderPosition) - EncoderCountThresholdToReverseDirection) 
							/(kDesiredIntakeArmEncoderSweep - EncoderCountThresholdToReverseDirection);
					intakeArmSpeed = mMaximumIntakeArmUpswingLiftMaxSpeed * factor;  // Percent
					if (showPrints) System.out.print("Phase R1: ");			
				} else {
					// after reachinig apogee, start at zero speed (letting momentum and gravity take over) 
					// then ramp up the reverse power until max braking speed (is reverse speed) 
					// applied near retracted landing position. 
					factor = (EncoderCountThresholdToReverseDirection - Math.abs(currentIntakeArmEncoderPosition))
							/EncoderCountThresholdToReverseDirection;
					intakeArmSpeed = mMaximumIntakeArmDownswingBrakingMaxSpeed * factor;  // Percent
					if (showPrints) System.out.print("Phase R2: ");
				} 
			}

		} else if (mDesiredIntakeArmPosition == IntakeArmPositionEnum.eDeployed) {

			if (getIntakeArmState() == IntakeArmPositionEnum.eDeployed) {
				intakeArmSpeed = -0.1;  // Apply persisting parking pressure, to counter robot motion dynamics
				if (showPrints) System.out.print("Phase D3: ");
			} else {
				mMaximumIntakeArmUpswingLiftMaxSpeed = -0.4;
				mMaximumIntakeArmDownswingBrakingMaxSpeed = 0.05;
				EncoderCountThresholdToReverseDirection = 4.5;

				if (Math.abs(currentIntakeArmEncoderPosition) < EncoderCountThresholdToReverseDirection) {
					// from retracted position, start with maximum lift speed but then ramp it down propotionaly to zero
					// along to apogee position.   Still a bit of momentum towards retracted position at apogee.
					factor = (EncoderCountThresholdToReverseDirection - Math.abs(currentIntakeArmEncoderPosition))
							/EncoderCountThresholdToReverseDirection;
					intakeArmSpeed = mMaximumIntakeArmUpswingLiftMaxSpeed * factor;  // Percent
					if (showPrints) System.out.print("Phase D1: ");
				} else {
					// after reachinig apogee, make zero speed (letting momentum and gravity take over) 
					// then ramp up the reverse power until max braking speed (is reverse speed) 
					// applied near deployed landing position. 
					factor = (Math.abs(currentIntakeArmEncoderPosition)-EncoderCountThresholdToReverseDirection)
							/(kDesiredIntakeArmEncoderSweep - EncoderCountThresholdToReverseDirection);
					intakeArmSpeed = mMaximumIntakeArmDownswingBrakingMaxSpeed * factor;  // Percent
					if (showPrints) System.out.print("Phase D2: ");
				} 
			}

		} else {
			factor = 0.0;
			intakeArmSpeed = 0.0;
			if (showPrints) System.out.print("Phase U0: "); // eUnknown
		}
		if (showPrints) System.out.println(	
			"IntakeArm: ENCODER: " + mIntakeArmEncoder.getPosition() +  
			" Desire: "+ mDesiredIntakeArmPosition +
			" SPD: " + intakeArmSpeed + 
			" Factor: " + factor + 
			" Sweep: " +  kDesiredIntakeArmEncoderSweep + 
			" Limit " + mIntakeRetractedLimitSwitch.get());

		if(mIntakeRetractedLimitSwitch.get()) {
			mIntakeArmEncoder.setPosition(0.0);
		} 

		if (kEnableMotors) mIntakeArmMotor.set(intakeArmSpeed);
	}

	public void setRollerMotorSpeed(double speed){
		mIntakeRollerMotor.set(speed); 
	}
	
	public void ingestNote() {
		double speed = kIngestNoteSpeed;
		if (!mExternalBeamBreaker.get()) {
			if (!mInternalBeamBreaker.get()) {
				speed = 0.0;
			} else {
				speed *= 0.5;
			}
		}   
		setRollerMotorSpeed(speed);
	}

	public void ejectNote() {
		if (!mExternalBeamBreaker.get() || !mInternalBeamBreaker.get()) {
			setRollerMotorSpeed(kEjectNoteSpeed);
		} else {
			setRollerMotorSpeed(0.0);
		}
	}

	public boolean isNoteCaptive(){
		return (!mExternalBeamBreaker.get() && !mInternalBeamBreaker.get());
	}

	public boolean isIntakeEmpty(){
		return (mExternalBeamBreaker.get() && mInternalBeamBreaker.get());
	}

	@Override
	public boolean checkSystem() {
		final boolean showPrints = false;		
		if (showPrints) System.out.println(
			"Intake ExternalBeamBreaker: " + 
			mExternalBeamBreaker.get() + 
			"  Intake InternalBeamBreaker: " + 
			mInternalBeamBreaker.get());
		return true;
	}

	@Override
	public void periodic() {
		runControlIntakeArmPosition();
	}

	//Add Shuffle board calls here
	public void outputTelemetry() {

	}

	public synchronized static IntakeSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeSubsystem();
		}
		return mInstance;
	}
}