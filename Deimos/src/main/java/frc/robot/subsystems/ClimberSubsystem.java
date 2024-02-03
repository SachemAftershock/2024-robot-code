//TODO: Bind Climber in, Climber out to buttons
package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem extends AftershockSubsystem {

	private static ClimberSubsystem mInstance;

	private double mClimberArmEncoderSetPoint = 0.5;
	private CANSparkMax mClimberArmMotor;
	private RelativeEncoder mClimberArmEncoder;
	private CANSparkMax mClimberRollerMotor;
	private double mClimberArmMotorSpeed = 0.05;
	private double mClimberRollerMotorSpeed = 0.05;
	private DigitalInput mClimberLimitSwitch;
	private boolean isClimberIn;
	private int mClimberArmMotorID = 0;
	private int mClimberRollerMotorID = 1;

	private ProfiledPIDController mClimberArmPidController;
	private Constraints mClimberArmPIDConstraints;
	private double mConstraintsMaxVelocity = 0;
	private double mConstraintsMaxAcceleration = 0;
	private double[] mClimberArmGains = { 0.4, 0, 0 };
	private final Encoder m_encoder = new Encoder(1, 0, false, Encoder.EncodingType.k4X);
	private ProfiledPIDController encoderPID;

	private ClimberSubsystem() {
		mClimberArmMotor = new CANSparkMax(mClimberArmMotorID, MotorType.kBrushless);
		mClimberRollerMotor = new CANSparkMax(mClimberRollerMotorID, MotorType.kBrushless);
		mClimberArmEncoder = mClimberArmMotor.getEncoder();
		mClimberArmEncoder.setPosition(0);
		mClimberLimitSwitch = new DigitalInput(0);
		isClimberIn = true;
		mClimberArmPIDConstraints = new Constraints(mConstraintsMaxVelocity, mConstraintsMaxAcceleration);
		mClimberArmPidController = new ProfiledPIDController(mClimberArmGains[0], mClimberArmGains[1],
				mClimberArmGains[0], mClimberArmPIDConstraints);
	}

	@Override
	public void initialize() {

	}

	public void move(double distance) {
		double speed = mClimberArmPidController.calculate(mClimberArmEncoder.getPosition(), distance);
		mClimberRollerMotor.set(speed);
	}

	// why do we have a periodic method if we are looping these checks as a command
	// @Override
	public void periodic() {

	}

	@Override
	public boolean checkSystem() {
		return true;
	}

	// what is telemetry @Override telemetry is used to view data logs if the robot
	// fails to perform as expected
	public void outputTelemetry() {

	}

	public synchronized static ClimberSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ClimberSubsystem();
		}
		return mInstance;
	}
}
