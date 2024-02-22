
package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.lib.PositionToVelocityProfiler;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ShooterConstants.*;
// import frc.robot.enums.ControlState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
	private CANcoder mAngleEncoder; 
	private RelativeEncoder mLeftShootEncoder;
	private RelativeEncoder mRightShootEncoder;
	private CANSparkMax mRightShootMotor;
	private DigitalInput mShooterLimitSwitch;
	private DigitalInput mBeamBreakerEnter;
	private DigitalInput mBeamBreakerLeave;

	private boolean mEjectNoteIntoAmp;

	private double mAngleEncoderCurrentPositionDegrees = 0;

	boolean showPrints = true;
	// states
	private ShooterAngleState mCurrentShooterAngleState = ShooterAngleState.eUnknown;

	private ShooterSubsystem() {
		mAngleShootMotor = new CANSparkMax(kAngleShootMotorID, MotorType.kBrushless);
		mBeamBreakerEnter = new DigitalInput(1); // TODO these were temporarily removed
		mBeamBreakerLeave = new DigitalInput(8);
		mLeftShootMotor = new CANSparkMax(kLeftShootMotorID, MotorType.kBrushless);
		mRightShootMotor = new CANSparkMax(kRightShootMotorID, MotorType.kBrushless);
		mLeftShootEncoder = mLeftShootMotor.getEncoder();
		mRightShootEncoder = mRightShootMotor.getEncoder();

		mAngleEncoder = new CANcoder(kAngleCANcoderID, "FRC263CANivore1");
  		mAngleEncoderCurrentPositionDegrees = mAngleEncoder.getPosition().getValueAsDouble() * 360 * -1.0;
		mLeftShootEncoder.setPosition(0);
		mRightShootEncoder.setPosition(0);
		mShooterLimitSwitch = new DigitalInput(kShooterLimitSwitchID);
		mEjectNoteIntoAmp = false;
		
	}

	@Override
	public void initialize() {
		if (mShooterLimitSwitch.get()) {
			setCurrentShooterAngleState(ShooterAngleState.eSpeaker);
		}

		mEjectNoteIntoAmp = false;
	}
	

	double jogAngle = ShooterAngleState.eSpeaker.getAngle();

	public boolean canFireIntoAmp() {
		return mEjectNoteIntoAmp;
	}

	public void setFireIntoAmp(boolean canFire) {
		mEjectNoteIntoAmp = canFire;
	}

	/**
	 * The positive direction fires notes upwards
	 * @param leftSpeed [-1.0, 1.0]
	 * @param rightSpeed [-1.0, 1.0]
	 */
	public void setShooterMotorSpeed(double leftSpeed, double rightSpeed) {
		// System.out.println("Called spin shooter motors");
		mLeftShootMotor.set(-1.0 * leftSpeed);
		mRightShootMotor.set(rightSpeed);
	}

	/**
	 * Sets rotational speed of the shooter. Recalibrates angle
	 * shooter if limit switch is hit (when angle arm hits the body of the bot)
	 * @param speed [-1.0, 1.0]. Positive input is outwards-facing.
	 */
	public void setAngleShooterMotorSpeed(double speed) {
		// Internally, it expects that a "negative" speed is back outward from the robot.
		// System.out.println("Sent speed:	" + speed);
		speed *= -1;
		mAngleShootMotor.set(speed);
	}
	
	/**
	 * Update current shooter state when we reach our setpoint, dictated
	 * by setDesiredShooterAngleState
	 * @param mCurrentShooterAngleState
	 */
	public void setCurrentShooterAngleState(ShooterAngleState mCurrentShooterAngleState) {
		// if (showPrints)
		// 	System.out.println("currentShooterAngleState := "+mCurrentShooterAngleState);
		this.mCurrentShooterAngleState = mCurrentShooterAngleState;
	}
	
	/**
	 * Updates state based on angle, then returns reading
	 * @return
	 */
	public ShooterAngleState getCurrentShooterAngleState() {
		// invert to make positive the upwards direction
		mAngleEncoderCurrentPositionDegrees = mAngleEncoder.getPosition().getValueAsDouble() * 360 * -1.0;
		if (mShooterLimitSwitch.get()) {
			setCurrentShooterAngleState(ShooterAngleState.eSpeaker);
		} else if (Math.abs(mAngleEncoderCurrentPositionDegrees - ShooterAngleState.eAmp.getAngle()) < 3.0) { // within 3 degrees
			setCurrentShooterAngleState(ShooterAngleState.eAmp);
		} else {
			setCurrentShooterAngleState(ShooterAngleState.eUnknown);
		}
		// if (showPrints)
		// 	System.out.println("returned currentShooterAngleState as "+mCurrentShooterAngleState);
	  	return mCurrentShooterAngleState;
	}
	
	private ShooterAngleState mDesiredShooterAngleState = ShooterAngleState.eSafeZone;

	/**
	 * Our setpoint for the shooter angle is an enum defined in ShooterAngleState.java
	 * @param mDesiredShooterAngleState
	 */
	public void setDesiredShooterAngleState(ShooterAngleState mDesiredShooterAngleState) {
		// if (showPrints)
		// 	System.out.println("mDesiredShooterAngleState := "+mDesiredShooterAngleState);
	  this.mDesiredShooterAngleState = mDesiredShooterAngleState;
	}
  
	public ShooterAngleState getDesiredShooterAngleState() {
		// if (showPrints)
			// System.out.println("returned desired as "+mDesiredShooterAngleState);
	  return mDesiredShooterAngleState;
	}

	private int counter = 0; // delay prints

	/**
	 * Non-PID setpoint chaser :)
	 */
	private void runShooterAngleSetpointChaser() {
		/**
		 * 3 Angles. Positive direction is inward towards the robot, negative direction is away.
		 * The shooter is physically angled at 34 degrees relative to the ground (just look at the
		 * smaller angle) --- we consider that as zero.
		 */
		counter++;

		double mDesiredEncoderValueDegrees = mDesiredShooterAngleState.getAngle();

		// Zero out when Shooter Angle Arm Limit Switch is pressed
		if (mShooterLimitSwitch.get()) {
			mAngleEncoder.setPosition(0);
		}

		mAngleEncoderCurrentPositionDegrees = mAngleEncoder.getPosition().getValueAsDouble() * 360 * -1.0;
		// Convert from rotations to degrees, then make the upward direction positive.

		double desiredSpeed = 0;
		switch (mDesiredShooterAngleState) {
			case eAmp:
				desiredSpeed = kSpeakerAngleAmpProfiler.calculate(mAngleEncoderCurrentPositionDegrees);
				break;
			case eSpeaker:
				desiredSpeed = kSpeakerAngleSpeakerProfiler.calculate(mAngleEncoderCurrentPositionDegrees);
				break;
			case eSafeZone: // TODO Are we using eSafeZone?
				desiredSpeed = 0; // a default value
				break;
			// m DESIRED shooter angle state will never be unknown
		}

		
		//System.out.println("current deg: " + mAngleEncoderCurrentPositionDegrees + "wanted v: " + desiredSpeed);
		// System.out.println("wanted v: " + desiredSpeed);
		setAngleShooterMotorSpeed(desiredSpeed);

	}

	public boolean isShooterAtSpeed() {
		return Math.abs(mLeftShootMotor.get() - kShootMotorShootingVelocity) < 0.1; 
	}

	public boolean isNoteLoaded() {
		return !(mBeamBreakerEnter.get());
	}

    @Override
	public void periodic(){
		runShooterAngleSetpointChaser();
		//call statecheck method, ... make statecheck call
		//if(mIntakeRetractedLimitSwitch.get()){
			//setCurrentIntakeState(IntakeState.eRetracted);
			//mIntakeArmEncoder.setPosition(0.0);//   .reset();
		// } TODO: FIX
		//System.out.println(mEjectNoteIntoAmp);
		//System.out.println("ACTUAL: " + mIntakeArmEncoder.getAbsolutePosition() + "-----Desired: " + mDesiredIntakeState.getDesiredPosition() + "-----Speed: " + mSpeed + "----error: " + mIntakeArmPidController.getPositionError() + "-----P*error: " + mIntakeArmPidController.getPositionError()* kIntakeArmGains[0]+ "LIMITSWITCH: " + mIntakeRetractedLimitSwitch.get());
	}



	// public ProfiledPIDController getShooterAnglePIDController() {
	// 	return mShooterAnglePIDController;
	// }

	@Override
	public boolean checkSystem() {
		return true;
	}

	@Override
	public void outputTelemetry() {
		// ShuffleboardTab tab = Shuffleboard.getTab("SubsystemShooter");
		// tab.add("Angle Encoder: ", mAngleEncoder.getRate()).getEntry();
		// tab.add("Left Shooter Encoder: ", mLeftShootEncoder.getVelocity()).getEntry();
		// tab.add("Right Shooter Encoder: ", mRightShootEncoder.getVelocity()).getEntry();

		// TODO:Dump current encoder speeds
		// private RelativeEncoder mAngleEncoder;
		// private RelativeEncoder mLeftShootEncoder;
		// private RelativeEncoder mRightShootEncoder;
	}

	public synchronized static ShooterSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ShooterSubsystem();
		}
		return mInstance;

	}
}
