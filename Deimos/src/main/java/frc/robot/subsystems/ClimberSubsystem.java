package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.AftershockSubsystem;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimberSubsystem extends AftershockSubsystem {
    private static ClimberSubsystem mInstance;

    private CANSparkMax mLeftClimberMotor;
    private CANSparkMax mRightClimberMotor;
    private DigitalInput mRightLimitSwitch;
    private DigitalInput mLeftLimitSwitch;

    private DoubleSupplier mDesiredLeftSpeedSupplier;
    private DoubleSupplier mDesiredRightSpeedSupplier;
    // private Double mDesiredLeftSpeedSupplier = () -> 0;
    // private Double mDesiredRightSpeedSupplier = () -> 0;

    ClimberSubsystem() {
        mLeftClimberMotor = new CANSparkMax(kLeftClimberMotorID, MotorType.kBrushless);
        mRightClimberMotor = new CANSparkMax(kRightClimberMotorID, MotorType.kBrushless);
        mLeftLimitSwitch = new DigitalInput(kLeftLimitSwitchID); // front of robot is intake
        mRightLimitSwitch = new DigitalInput(kRightLimitSwitchID);
        mDesiredLeftSpeedSupplier = () -> 0;
        mDesiredRightSpeedSupplier = () -> 0;
    }

	public enum climberMotorToSpinEnum { right, left, both };

    @Override
    public void initialize() {
        
    }

    public void setClimberMotorSpeed(DoubleSupplier desiredSpeedSup, climberMotorToSpinEnum motorToSpin) {
        switch (motorToSpin) {
            case right:
                mDesiredRightSpeedSupplier = desiredSpeedSup;
                break;
            case left:
                mDesiredLeftSpeedSupplier = desiredSpeedSup;
                break;
            case both:
                // mLeftClimberMotor.set(speed);
                // mRightClimberMotor.set(speed);
                break;
        }
        
    }
    @Override
    public void periodic(){
        double desiredLeftSpeed = mDesiredLeftSpeedSupplier.getAsDouble();
        double desiredRightSpeed = mDesiredRightSpeedSupplier.getAsDouble();
        if (mRightLimitSwitch.get() && desiredRightSpeed > 0) {
            desiredRightSpeed = 0;
        }
        if (mLeftLimitSwitch.get() && desiredLeftSpeed > 0) {
            desiredLeftSpeed = 0;
        }
        // System.out.println("L "+desiredLeftSpeed + " R "+desiredRightSpeed);
        mLeftClimberMotor.set(desiredLeftSpeed);
        mRightClimberMotor.set(desiredRightSpeed);
    }
    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

    public synchronized static ClimberSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ClimberSubsystem();
		}
		return mInstance;
	}
}
