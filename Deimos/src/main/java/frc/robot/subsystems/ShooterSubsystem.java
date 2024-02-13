//TODO: Set up angle encoder right

package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ShooterConstants.*;
// import frc.robot.enums.ControlState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;

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
	private Encoder mAngleEncoder; // FIXME we will need to call .reset() for zeroing
	private RelativeEncoder mLeftShootEncoder;
	private RelativeEncoder mRightShootEncoder;
	private CANSparkMax mRightShootMotor;
	private DigitalInput mShooterLimitSwitch;
	private DigitalInput mBeamBreakerEnter;
	private DigitalInput mBeamBreakerLeave;

	private boolean mEnableMotors = true;
	// PID for shooting
	private ProfiledPIDController mRightShooterPIDController;
	private Constraints mRightShooterPIDConstraints;
	private ProfiledPIDController mLeftShooterPIDController;
	private Constraints mLeftShooterPIDConstraints;
	private double mShooterConstraintsMaxAcceleration = kShooterConstraintsMaxAcceleration;
	private double[] mShooterGains = kShooterGains;

	// PID for angle stuff
	private double mAngleShooterMotorSpeed;
	private ProfiledPIDController mShooterAnglePIDController;
	private TrapezoidProfile.Constraints mShooterAnglePIDConstraints;
	private double[] mAngleGains = kAngleGains;
	double leftSpeed, rightSpeed;

	private ShooterSubsystem() {
		mAngleShootMotor = new CANSparkMax(kAngleShootMotorID, MotorType.kBrushless);
		// mBeamBreakerEnter = new DigitalInput(kBeamBreakerEnterID); // TODO these were temporarily removed
		// mBeamBreakerLeave = new DigitalInput(kBeam`BreakerLeaveID);
		mLeftShootMotor = new CANSparkMax(kLeftShootMotorID, MotorType.kBrushless);
		mRightShootMotor = new CANSparkMax(kRightShootMotorID, MotorType.kBrushless);
		mLeftShootEncoder = mLeftShootMotor.getEncoder();
		mRightShootEncoder = mRightShootMotor.getEncoder();
		mAngleEncoder = new Encoder(0, 1); // FIXME wrong encoder
		mLeftShootEncoder.setPosition(0);
		mRightShootEncoder.setPosition(0);
		mShooterLimitSwitch = new DigitalInput(kShooterLimitSwitchID);
		mLeftShooterPIDConstraints = new Constraints(kConstraintsMaxVelocity,
				mShooterConstraintsMaxAcceleration);
		mLeftShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2],
				mLeftShooterPIDConstraints);
		mRightShooterPIDConstraints = new Constraints(kConstraintsMaxVelocity,
				mShooterConstraintsMaxAcceleration);
		mRightShooterPIDController = new ProfiledPIDController(mShooterGains[0], mShooterGains[1], mShooterGains[2],
				mRightShooterPIDConstraints);
		mShooterAnglePIDConstraints = new TrapezoidProfile.Constraints(kConstraintsMaxVelocity,
				kAngleMaxAcceleration);
		mShooterAnglePIDController = new ProfiledPIDController(mAngleGains[0], mAngleGains[1], mAngleGains[2],
				mShooterAnglePIDConstraints);
	}

	@Override
	public void initialize() {
		mAngleEncoder.reset();

	}

	double jogAngle = ShooterAngleState.eSpeaker.getAngle();

	public void manualJogShooter(double speed) {
		mShooterAnglePIDController.calculate(speed);
	}

	/**
	 * The positive direction fires notes upwards
	 * @param leftSpeed [-1.0, 1.0]
	 * @param rightSpeed [-1.0, 1.0]
	 */
	public void spinShooterMotors(double leftSpeed, double rightSpeed) {
		// startRollerMotor method or something here
		System.out.println("Called spin shooter motors");
		// TODO pid not working, stuck at 0
		// leftSpeed = mLeftShooterPIDController.calculate(mLeftShootMotor.getEncoder().getVelocity(), leftSpeed);
		// rightSpeed = mRightShooterPIDController.calculate(mRightShootMotor.getEncoder().getVelocity(), rightSpeed);
		mLeftShootMotor.set(-1.0 * leftSpeed);
		mRightShootMotor.set(rightSpeed); // TODO: USE PID so that speed is consistent despite battery charge/weakness
	}

	/**
	 * Sets rotational speed of the shooter. Recalibrates angle
	 * shooter if limit switch is hit (when angle arm hits the body of the bot)
	 * @param speed [-1.0, 1.0]. Positive is inward-facing.
	 */
	public void setAngleShooterMotorSpeed(double speed) {
		// System.out.println(speed);
		// if (mShooterLimitSwitch.get()) {   // if pressed,
			// System.out.println("mShooterLimitSwitch was pressed");
			// mAngleShootMotor.set(0); // stop the motor
			// mAngleEncoder.reset(); 		   // zero out the angle encoder (stop error accumulate)
		// } else {                           // otherwise,
			// mAngleShootMotor.set(speed);   // set speed as normal
			counter++;
			if (counter%50==0||counter%50==1)
			System.out.println("Wanted speed: "+mAngleShooterMotorSpeed);
		// }
	}
	/**
	 * Assume that we start in eSafeZone, which is all the way down
	 */
	private ShooterAngleState mCurrentShooterAngleState = ShooterAngleState.eSafeZone;
	
	/**
	 * Update current shooter state when we reach our setpoint, dictated
	 * by setDesiredShooterAngleState
	 * @param mCurrentShooterAngleState
	 */
	public void setCurrentShooterAngleState(ShooterAngleState mCurrentShooterAngleState) {
		this.mCurrentShooterAngleState = mCurrentShooterAngleState;
	}
	
	public ShooterAngleState getCurrentShooterAngleState() {
	  return mCurrentShooterAngleState;
	}
	
	private ShooterAngleState mDesiredShooterAngleState;

	/**
	 * Our setpoint for the shooter angle is an enum defined in ShooterAngleState.java
	 * @param mDesiredShooterAngleState
	 */
	public void setDesiredShooterAngleState(ShooterAngleState mDesiredShooterAngleState) {
	  this.mDesiredShooterAngleState = mDesiredShooterAngleState;
	}
  
	public ShooterAngleState getDesiredShooterAngleState() {
	  return mDesiredShooterAngleState;
	}

	// Should be called continuously, returns true when error is less than a certain
	// epsilon
	// public boolean runShooterPID() {
	// 	double mDesiredEncoderValueDegrees;
	// 	// if (getControlState() != ControlState.eManualControl) {
	// 		mDesiredEncoderValueDegrees = getDesiredShooterAngleState().getAngle();
	// 	// } else {
	// 	// 	mDesiredEncoderValueDegrees = jogAngle;
	// 	// }
	// 	double speed = mShooterAnglePIDController.calculate(mAngleEncoder.getDistance(), mDesiredEncoderValueDegrees);
	// 	setAngleShooterMotorSpeed(speed);
	// 	final double mArmAngleEpsilon = 0.001;
	// 	if (Math.abs(mShooterAnglePIDController.getPositionError()) < mArmAngleEpsilon) {

	// 		mShooterAnglePIDController.setP(kShooterAngleSetPWhenBelowEpsilon); // Test when we have access to robot arm
	// 																			// and which values best
	// 		// counteract gravity
	// 		return true;
	// 	}
	// 	return false;
	// }
	private int counter = 0;
	public void runShooterAnglePID(){
		double mDesiredEncoderValueDegrees = mDesiredShooterAngleState.getAngle();
		mShooterAnglePIDController.setGoal(mDesiredEncoderValueDegrees);
		counter++;
		/**
		 * Reads shooter arm rotation with a precision of ~4 degrees.
		 * The "zero", which is when the shooter arm is down (eSpeaker),
		 * is generally between 0 and -1.0 --- e.g. -.308, -.43, etc.
		 * 
		 * We acquired the magic number 5.68888 with this:
		 * counts per rev (https://www.revrobotics.com/content/docs/REV-11-1271-DS.pdf)
		 * divided by 360 divided by 4 (for 4x encoding, we think)
		 * Essentially: 8192/360/4
		 */
		double mAngleEncoderCurrentPositionDegrees = mAngleEncoder.getDistance() / 5.688888;
		
		mAngleShooterMotorSpeed = mShooterAnglePIDController.calculate(
			/*TODO FIXME this is rotations. are we comparing to angles or radians */
			mAngleEncoderCurrentPositionDegrees, 
			mDesiredEncoderValueDegrees // this is also in degrees
		);

		counter++;
		if (counter % 20 == 0) {
			System.out.println("|||||||||CurrentShootState "+ mCurrentShooterAngleState);
			System.out.println("WantedShootState "+ mDesiredShooterAngleState);
			System.out.println("getDistance() / 5.68 = "+mAngleEncoderCurrentPositionDegrees);
			System.out.println("SPEED IN PID FILTER: " + mAngleShooterMotorSpeed);
			System.out.println("RAW RATE "+mAngleEncoder.getRate());
			// System.out.println("--------Desired State encoder: " + mDesiredEncoderValueDegrees);
		}

		if(Math.abs(mShooterAnglePIDController.getPositionError()) < kShooterAngleEpsilon){
			// Stops motors if reached setpoint. TODO: make actual espilon
			System.out.println("||||||||||BELOW EPSILON!!!");
			System.out.println("---CurrentShootState "+ mCurrentShooterAngleState);
			System.out.println("---WantedShootState "+ mDesiredShooterAngleState);
			System.out.println("---getDistance() / 5.68 = "+mAngleEncoderCurrentPositionDegrees);
			System.out.println("---SPEED IN PID FILTER: " + mAngleShooterMotorSpeed);
			System.out.println("--- RAW RATE "+mAngleEncoder.getRate());
			System.out.println("---below epsilon at "+mDesiredEncoderValueDegrees);
			mAngleShooterMotorSpeed = 0;
			System.out.println("---ZEROED SPEED: " + mAngleShooterMotorSpeed);
			if(mEnableMotors) setAngleShooterMotorSpeed(mAngleShooterMotorSpeed);
			setCurrentShooterAngleState(mDesiredShooterAngleState);
		}
		if(mEnableMotors) setAngleShooterMotorSpeed(mAngleShooterMotorSpeed);








	
	
	}


	public ProfiledPIDController getShooterAnglePIDController() {
		return mShooterAnglePIDController;
	}

	@Override
	public void outputTelemetry() {
		ShuffleboardTab tab = Shuffleboard.getTab("SubsystemShooter");
		tab.add("Angle Encoder: ", mAngleEncoder.getRate()).getEntry();
		tab.add("Left Shooter Encoder: ", mLeftShootEncoder.getVelocity()).getEntry();
		tab.add("Right Shooter Encoder: ", mRightShootEncoder.getVelocity()).getEntry();
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
