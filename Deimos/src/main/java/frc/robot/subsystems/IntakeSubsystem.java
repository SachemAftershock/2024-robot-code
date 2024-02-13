//TODO: Bind intake in, intake out to buttons
package frc.robot.subsystems;


import frc.lib.AftershockSubsystem;
import frc.lib.PID;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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
	final private boolean mEnableMotors = false;
	private DigitalInput mIntakeRetractedLimitSwitch;
	
	//private PIDController mIntakeArmPidController;
	private ProfiledPIDController mIntakeArmPidController;
	private TrapezoidProfile.Constraints mIntakeArmPIDConstraints;

	//private final Encoder mIntakeArmEncoder;
	private double mSpeed = 0.0;
	//rec  3, trans 4 BEAM BREAKER

	//private IntakeState mDesiredIntakeState;

	public final double kIntakeConstraintsMaxVelocity = 0.3;
	public final double kIntakeConstraintsMaxAcceleration = 0.05;

	public enum  IntakeArmPositionEnum { eUnknown, eDeployed, eRetracted };
	private IntakeArmPositionEnum mDesiredIntakeArmPosition = IntakeArmPositionEnum.eUnknown;
	
	private IntakeSubsystem() {
		
		//mIntakeArmEncoder.setPositionConversionFactor(2000);
		mIntakeArmMotor = new CANSparkMax(kIntakeArmMotorID, MotorType.kBrushless);
		mIntakeArmEncoder = mIntakeArmMotor.getEncoder(); //TODO: make encoder value into constants //TODO: change from DIO to alternate encoder port
		mIntakeArmEncoder.setPosition(0.0);
		
		mIntakeRollerMotor = new CANSparkMax(kIntakeRollerMotorID, MotorType.kBrushless);

		mExternalBeamBreaker = new DigitalInput(kExternalBeamBreakerID); //EXTERNAL B1
		mInternalBeamBreaker = new DigitalInput(kInternalBeamBreakerID); //INTERNAL B2

		mIntakeRetractedLimitSwitch = new DigitalInput(kIntakeLimitSwitchID);
		mIntakeArmPIDConstraints = new TrapezoidProfile.Constraints(0.4, 0.25);
		mIntakeArmPidController = new ProfiledPIDController(kIntakeArmGains[0], kIntakeArmGains[1], kIntakeArmGains[2], mIntakeArmPIDConstraints);
	}
	
	@Override
	public void initialize() {
		mIntakeArmMotor.setIdleMode(IdleMode.kBrake);
	}

	public void RetractIntake() {
		mDesiredIntakeArmPosition = IntakeArmPositionEnum.eRetracted;
	}

	public void DeployIntake() {
		mDesiredIntakeArmPosition = IntakeArmPositionEnum.eDeployed;
	}

	public IntakeArmPositionEnum getIntakeArmState() {
		IntakeArmPositionEnum currentIntakeArmPosition;
		if(!mIntakeRetractedLimitSwitch.get())
			currentIntakeArmPosition = IntakeArmPositionEnum.eRetracted;
		else if ((mIntakeArmEncoder.getPosition() < 1.0) || (mIntakeArmEncoder.getPosition() > 7.0)) 
			currentIntakeArmPosition = IntakeArmPositionEnum.eDeployed;
		else
			currentIntakeArmPosition = IntakeArmPositionEnum.eUnknown;
		return currentIntakeArmPosition;
	}

	
	public void runControlIntakeArmPosition(){
		final boolean showPrints = true;
		
		double mDesiredIntakeArmEncoderSweep = -1;
		double mMaximumIntakeArmUpswingLiftMaxSpeed = -1;
		double mMaximumIntakeArmDownswingBrakingMaxSpeed = -1;
		double EncoderCountThresholdToReverseDirection = -1;
	
		double currentIntakeArmEncoderPosition = mIntakeArmEncoder.getPosition();

		//System.out.println("LIMIT SWITCH: " + mIntakeRetractedLimitSwitch.get());
		double intakeArmSpeed = 0;
		double factor = 0;

		if (mDesiredIntakeArmPosition == IntakeArmPositionEnum.eRetracted) {

			mDesiredIntakeArmEncoderSweep = 8.0;
			mMaximumIntakeArmUpswingLiftMaxSpeed = 0.4;
			mMaximumIntakeArmDownswingBrakingMaxSpeed = -0.05;
			EncoderCountThresholdToReverseDirection = 6.5;

		if (Math.abs(currentIntakeArmEncoderPosition) < EncoderCountThresholdToReverseDirection) {
				// from deployed position start with maxium lift speed but then ramp it down propotionaly
				// to full swing, but only up to the apogee.   So still a bit of momentum towards 
				// Retracted position at apogee.
				factor = (EncoderCountThresholdToReverseDirection - currentIntakeArmEncoderPosition)/(EncoderCountThresholdToReverseDirection);
				intakeArmSpeed = mMaximumIntakeArmUpswingLiftMaxSpeed * factor;  // Percent
				if (showPrints) System.out.print("Phase 1: ");
			} else {
				// after reachinig apogee, make zero speed (letting momentum and gravity take over) 
				// then ramp up the reverse power until max braking speed (is negative speed) 
				// applied near retracted landing position. 
				factor = Math.abs((EncoderCountThresholdToReverseDirection - currentIntakeArmEncoderPosition)/(mDesiredIntakeArmEncoderSweep - Math.abs(EncoderCountThresholdToReverseDirection)));
				intakeArmSpeed = mMaximumIntakeArmDownswingBrakingMaxSpeed * factor;  // Percent
				if (showPrints) System.out.print("Phase 2: ");
			} 

		} else if (mDesiredIntakeArmPosition == IntakeArmPositionEnum.eDeployed) {

			mDesiredIntakeArmEncoderSweep = 8.0;
			mMaximumIntakeArmUpswingLiftMaxSpeed = 0.4;
			mMaximumIntakeArmDownswingBrakingMaxSpeed = -0.05;
			EncoderCountThresholdToReverseDirection = 6.5;

			if (Math.abs(currentIntakeArmEncoderPosition) > EncoderCountThresholdToReverseDirection) {
				// from retracted position start with maxium lift speed but then ramp it down propotionaly
				// to full swing, but only up to the apogee.   So still a bit of momentum towards 
				// deployed position at apogee.
				factor = -Math.abs((EncoderCountThresholdToReverseDirection - currentIntakeArmEncoderPosition)/(mDesiredIntakeArmEncoderSweep - Math.abs(EncoderCountThresholdToReverseDirection)));
				intakeArmSpeed = mMaximumIntakeArmDownswingBrakingMaxSpeed * factor;  // Percent
				if (showPrints) System.out.print("Phase 3: ");			
			} else {
				// after reachinig apogee, make zero speed (letting momentum and gravity take over) 
				// then ramp up the reverse power until max braking speed (is negative speed) 
				// applied near deployed landing position. 
				factor = -(EncoderCountThresholdToReverseDirection - currentIntakeArmEncoderPosition)/(EncoderCountThresholdToReverseDirection);
				intakeArmSpeed = mMaximumIntakeArmUpswingLiftMaxSpeed * factor;  // Percent
				if (showPrints) System.out.print("Phase 4: ");
			} 

		} else {
			intakeArmSpeed = 0.0;
			if (showPrints) System.out.print("Phase 0: ");
		}
		if (showPrints) System.out.println("CAL: ENCODER: " + mIntakeArmEncoder.getPosition() +  " SPD: " + intakeArmSpeed + " Factor: " + factor + " Desire: "+ mDesiredIntakeArmPosition +" Sweep: " +  mDesiredIntakeArmEncoderSweep + " Limit " + mIntakeRetractedLimitSwitch.get());

		if(mIntakeRetractedLimitSwitch.get()) {
			mIntakeArmEncoder.setPosition(0.0);
		} 

		if (mEnableMotors) mIntakeArmMotor.set(intakeArmSpeed);
	}

	public void setRollerMotorSpeed(double speed){
		mIntakeRollerMotor.set(speed); 
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