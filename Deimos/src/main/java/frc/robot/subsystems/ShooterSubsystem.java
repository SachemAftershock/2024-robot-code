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
	public void setShooterMotorSpeed(double leftSpeed, double rightSpeed) {
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
	 * @param speed [-1.0, 1.0]. Positive input is outwards-facing.
	 */
	public void setAngleShooterMotorSpeed(double speed) {
		// Internally, it expects that negative is back out from the robot.
		System.out.println("Sent speed:	" + speed);
		speed *= -1;
		// System.out.println(speed);
		// if (mShooterLimitSwitch.get()) {   // if pressed,
			// System.out.println("mShooterLimitSwitch was pressed");
			// mAngleShootMotor.set(0); // stop the motor
			// mAngleEncoder.reset(); 		   // zero out the angle encoder (stop error accumulate)
		// } else {                           // otherwise,
			mAngleShootMotor.set(speed);   // set speed as normal
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
	
	private ShooterAngleState mDesiredShooterAngleState = ShooterAngleState.eSafeZone;

	/**
	 * Our setpoint for the shooter angle is an enum defined in ShooterAngleState.java
	 * @param mDesiredShooterAngleState
	 */
	public void setDesiredShooterAngleState(ShooterAngleState mDesiredShooterAngleState) {
		System.out.println("Desired shooter state being called");
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

	/**
	 * Non-PID setpoint chaser :)
	 * 
	 */

	 /**
	  * helper method for picking the speed based on a heatfield (interval (its just if statements))
	  * @param input
	  * @param min
	  * @param max
	  * @param speedWeWant
	  */
	private void speedDecider(double input, double min, double max, double speedWeWant) {
		if (min <= input && input < max) {
			System.out.printf("SpeedDecider accepted: %f %f %f\n", min, input, max);
			setAngleShooterMotorSpeed(speedWeWant);
		} else {
			System.out.printf("SpeedDecider rejected: %f %f %f\n", min, input, max);
		}
	}

	/**
	 * Instead of using a PID, use a big array of intervals for speeds
	 * 
	 * <pre>
	 * {@code
	 * {
	 *   {minimumPos, maximumPos, desiredSpeed}, // desiredSpeed [-1.0, 1.0]
	 *   {minimumPos, maximumPos, desiredSpeed},
	 *   ...
	 *}
	 * </pre>
	 * 
	 * @param differenceFromSetpoint filtered through the heatfield
	 * @param twoDimArr see above
	 */
	private void getSpeedFromPositionWith2DArray(double differenceFromSetpoint, double[][] twoDimArr) {
		for (int i = 0; i < twoDimArr.length; i++) {
			double[] intervalSet = twoDimArr[i];
			double low = intervalSet[0];
			double hi = intervalSet[1];
			if (low <= differenceFromSetpoint && differenceFromSetpoint < hi) {
				break;
			}
		}
	}

	public void runShooterAngleSetpointChaser() {
		/**
		 * 3 Angles. Positive direction is inward towards the robot, negative direction is away.
		 * The shooter is physically angled at 34 degrees relative to the ground (just look at the
		 * smaller angle) --- we consider that as zero.
		 */

		double mDesiredEncoderValueDegrees = mDesiredShooterAngleState.getAngle();
		double mAngleEncoderCurrentPositionDegrees = -1.0 * mAngleEncoder.getDistance() / 5.688888;
		mShooterAnglePIDController.setGoal(mDesiredEncoderValueDegrees);

		//							like 20							like 0
		double diffFromSetpointDegrees = mDesiredEncoderValueDegrees - mAngleEncoderCurrentPositionDegrees;
		System.out.println("currentAngle: " + mAngleEncoderCurrentPositionDegrees);
		System.out.println("desiredAngle: " + mDesiredEncoderValueDegrees);

		speedDecider(mAngleEncoderCurrentPositionDegrees, -2, 20, 0.5);
		speedDecider(mAngleEncoderCurrentPositionDegrees, 20, 30, 0.2);
		speedDecider(mAngleEncoderCurrentPositionDegrees, 30, 37.5, .15);
		speedDecider(mAngleEncoderCurrentPositionDegrees, 37.5, 42.5, .07);
		speedDecider(mAngleEncoderCurrentPositionDegrees, 42.5, 45, -0.05);
		speedDecider(mAngleEncoderCurrentPositionDegrees, 45, 60, -0.2);
		speedDecider(mAngleEncoderCurrentPositionDegrees, 60, 80, -0.5);
	}

    @Override
	public void periodic(){
		//call statecheck method, ... make statecheck call
		//if(mIntakeRetractedLimitSwitch.get()){
			//setCurrentIntakeState(IntakeState.eRetracted);
			//mIntakeArmEncoder.setPosition(0.0);//   .reset();
		// } TODO: FIX
		//System.out.println();
		//System.out.println("ACTUAL: " + mIntakeArmEncoder.getPosition() + "-----Desired: " + mDesiredIntakeState.getDesiredPosition() + "-----Speed: " + mSpeed + "----error: " + mIntakeArmPidController.getPositionError() + "-----P*error: " + mIntakeArmPidController.getPositionError()* kIntakeArmGains[0]+ "LIMITSWITCH: " + mIntakeRetractedLimitSwitch.get());
	}



	public ProfiledPIDController getShooterAnglePIDController() {
		return mShooterAnglePIDController;
	}

	@Override
	public boolean checkSystem() {
		return true;
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
