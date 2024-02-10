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
	private boolean mEnableMotors = true;
	private DigitalInput mIntakeRetractedLimitSwitch;
	
	//private PIDController mIntakeArmPidController;
	private ProfiledPIDController mIntakeArmPidController;
	private TrapezoidProfile.Constraints mIntakeArmPIDConstraints;

	//private final Encoder mIntakeArmEncoder;
	private double mSpeed = 0.0;
	//rec  3, trans 4 BEAM BREAKER

	//private IntakeState mDesiredIntakeState;

	// Intake arm position Concept of Operations (CONOPS)
	// When robot is powered on, the intake arm is using a
	// relative encoder, so does not know its position.  The arm
	// is pulled down by gravity to either of its extreme positions.
	// The approach is to use a limit switch at the Retracted position 
	// to detect that position, and reset the encoder to zero counts.
	// If limit switch is detected upon auton or tele enable, then 
	// calibration is complete.  If not, then assume fully deployed.
	// That case we empirically measured the count sweep for the full
	// swing motion of the intake arm as about 8.0, starting there
	// to then travel/decrement down to encoder position count of zero-ish.
	// We make the motor speed move the arm to the retracted position. 
	// We move slow because are not totally sure where we are or how 
	// much power will prevent a slam.  Gravity is still an issue, so 
	// we power higher and diminish during the lift to apogee, then
	// reverse power and start from zero speed up to a workable 
	// soft landing spead, then zero the power. 
	// The encoder threshold to switch motor direction is 6.5.

	private boolean mIntakeCalibrated = false;
	private boolean mCalibrateNeverCalled = true;
	private double mCurrentCalibrateCount = 0;
	private final double mDesiredCalibrateCountSweep = 8.0;
	private double mDesiredCalibrateCount = 8.0;
	private double mMaximumCalibrationUpswingLiftMaxSpeed = 0.4;
	private double mMaximumCalibrationDownswingBrakingMaxSpeed = -0.05;
	private double EncoderCountThresholdToReverseDirection = 6.5;

	public final double kIntakeConstraintsMaxVelocity = 0.3;
	public final double kIntakeConstraintsMaxAcceleration = 0.05;

	private final double kIntakeArmEpsilon  = 0.2;

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
		//mIntakeArmPIDConstraints = new TrapezoidProfile.Constraints(kIntakeConstraintsMaxVelocity, kIntakeConstraintsMaxAcceleration);
		mIntakeArmPIDConstraints = new TrapezoidProfile.Constraints(0.1, 0.05);
		mIntakeArmPidController = new ProfiledPIDController(kIntakeArmGains[0], kIntakeArmGains[1], kIntakeArmGains[2], mIntakeArmPIDConstraints);
		//mIntakeArmPidController = new PIDController(kIntakeArmGains[0], kIntakeArmGains[1], kIntakeArmGains[2]);
	}
	public void resetCalibration(){
		mIntakeCalibrated = false;
		mCalibrateNeverCalled = true;
	}

	@Override
	public void initialize() {
		mIntakeArmMotor.setIdleMode(IdleMode.kBrake);
	}

	// public void setDesiredState(IntakeState mDesiredIntakeState){
	// 	this.mDesiredIntakeState = mDesiredIntakeState;
	// }
	private IntakeState mCurrentIntakeState;

	public void setCurrentIntakeState(IntakeState CurrentIntakeState) {
		mCurrentIntakeState = CurrentIntakeState;
	  }
	
	  public IntakeState getIntakeState() {
		return mCurrentIntakeState;
	  }
	
	  private IntakeState mDesiredIntakeState = IntakeState.eSafeShooterMovement;
	
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
		double mDesiredEncoderValue = mDesiredIntakeState.getDesiredPosition();
		mIntakeArmPidController.setGoal(mDesiredEncoderValue);

		mSpeed = mIntakeArmPidController.calculate(mIntakeArmEncoder.getPosition(), mDesiredEncoderValue);

		
		// System.out.println("-------Desired State encoder: " + mDesiredEncoderValue);
		// System.out.println("-------Current State encoder IntakeState: " + mCurrentIntakeState.getDesiredPosition());
		// System.out.println("-------Desired State encoder IntakeState: " + mDesiredIntakeState.getDesiredPosition());
		// System.out.println("CURRENT SPEED: " + speed);
		if(Math.abs(mIntakeArmPidController.getPositionError()) < kIntakeArmEpsilon){//TODO: make espilon
			mSpeed = 0;
			if(mEnableMotors) mIntakeArmMotor.set(mSpeed);
			setCurrentIntakeState(mDesiredIntakeState);
			//return true;
		}
		if(mEnableMotors) mIntakeArmMotor.set(mSpeed);
		// if(Math.abs(mIntakeArmPidController.getPositionError()) < 0.1){// TODO: add actual epsilon
		// 	setCurrentIntakeState(mDesiredIntakeState);
		// }

		if(mIntakeRetractedLimitSwitch.get()){
			setDesiredIntakeState(IntakeState.eRetracted);
			setCurrentIntakeState(IntakeState.eRetracted);

			mIntakeArmEncoder.setPosition(0.0);
		} 

		// if(getDesiredIntakeState()==IntakeState.eRetracted && getIntakeState()==IntakeState.eRetracted){
		// 	mSpeed = 0;
		// 	mIntakeArmMotor.set(mSpeed);
		// 	setCurrentIntakeState(IntakeState.eRetracted);
		// }

	
	}

	/*At the first time the robot turns on, the home state should be
	 * eRetracted, so the limit switch is checked to see if it is in the right position otherwise 
	 * sets the speed slow and moves to the limit switch
	 */
	public boolean runCalibrateIntake(){

		mCurrentCalibrateCount = mIntakeArmEncoder.getPosition();
		if (mCalibrateNeverCalled) { 
			System.out.println("CAL NEVER CALLED");
			mDesiredCalibrateCount = mCurrentCalibrateCount - mDesiredCalibrateCountSweep;
			mCalibrateNeverCalled = false;
		}

		//System.out.println("LIMIT SWITCH: " + mIntakeRetractedLimitSwitch.get());
		double calibrationRetractionSpeed = 0;
		double factor = 0;
		if (Math.abs(mCurrentCalibrateCount) < EncoderCountThresholdToReverseDirection) {
			// from deployed position start with maxium lift speed but then ramp it down propotionaly
			// to full swing, but only up to the apogee.   So still a bit of momentum towards 
			// Retracted position at apogee.
			factor = (EncoderCountThresholdToReverseDirection - mCurrentCalibrateCount)/(EncoderCountThresholdToReverseDirection);
			calibrationRetractionSpeed = mMaximumCalibrationUpswingLiftMaxSpeed * factor;  // Percent
			System.out.print("Phase 1: ");
		}
		else{
			// after reachinig apogee, make zero speed (letting momentum and gravity take over) 
			// then ramp up the reverse power until max braking speed (is negative speed) 
			// applied near retracted landing position. 
			factor = Math.abs((EncoderCountThresholdToReverseDirection - mCurrentCalibrateCount)/(mDesiredCalibrateCountSweep - Math.abs(EncoderCountThresholdToReverseDirection)));
			calibrationRetractionSpeed = mMaximumCalibrationDownswingBrakingMaxSpeed * factor;  // Percent
			System.out.print("Phase 2: ");
		}
		System.out.println("CAL: ENCODER: " + mIntakeArmEncoder.getPosition() +  " SPD: " + calibrationRetractionSpeed + " Factor: " + factor + " Desire: "+ mDesiredCalibrateCount +" Sweep: " +  mDesiredCalibrateCountSweep + " Limit " + mIntakeRetractedLimitSwitch.get());

		if(!mIntakeRetractedLimitSwitch.get()){
			if(mEnableMotors) {
				//System.out.println("ENCODER: " + mIntakeArmEncoder.getPosition());
				mIntakeArmMotor.set(calibrationRetractionSpeed);
			}
		}else{
			double speed = 0;
			if(mEnableMotors) mIntakeArmMotor.set(speed);
			setDesiredIntakeState(IntakeState.eRetracted);
			setCurrentIntakeState(IntakeState.eRetracted);
			mIntakeArmEncoder.setPosition(0.0);
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
		while(mIntakeArmEncoder.getPosition() < -1.0)
		{
		// if(speed > -1.0){
		// 	speed -= 0.1;	
		// }
		// else
		// {
		// 	speed = -1.0;
		// }
		mIntakeArmMotor.set(0.3);
		}
		mIntakeArmMotor.set(0);
		//mIntakeArmMotor.set(speed);
		//System.out.println("INTAKE ARM MOTOR SPEED: " + speed + "-------------");
		//System.out.println("ENCODER VALUES: " + mIntakeArmEncoder.getPosition() + "-------");

	}
	//TEMP
	public void setNegIntakeArmMotorSpeed() {
		while(mIntakeArmEncoder.getPosition() > -4.0)
		{
		// if(speed > -1.0){
		// 	speed -= 0.1;	
		// }
		// else
		// {
		// 	speed = -1.0;
		// }
		mIntakeArmMotor.set(-0.3);
		}
		mIntakeArmMotor.set(0);
		//System.out.println("INTAKE ARM MOTOR SPEED: " + speed + "-------------");
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
		//if(mIntakeRetractedLimitSwitch.get()){
			//setCurrentIntakeState(IntakeState.eRetracted);
			//mIntakeArmEncoder.setPosition(0.0);//   .reset();
		// } TODO: FIX
		runIntakePID();
		//System.out.println();
		//System.out.println("ACTUAL: " + mIntakeArmEncoder.getPosition() + "-----Desired: " + mDesiredIntakeState.getDesiredPosition() + "-----Speed: " + mSpeed + "----error: " + mIntakeArmPidController.getPositionError() + "-----P*error: " + mIntakeArmPidController.getPositionError()* kIntakeArmGains[0]+ "LIMITSWITCH: " + mIntakeRetractedLimitSwitch.get());
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