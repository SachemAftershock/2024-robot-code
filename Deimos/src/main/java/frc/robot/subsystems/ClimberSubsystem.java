package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.AftershockSubsystem;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem extends AftershockSubsystem {
    private static ClimberSubsystem mInstance;

    private CANSparkMax mLeftClimberMotor;
    private CANSparkMax mRightClimberMotor;

    ClimberSubsystem() {
        mLeftClimberMotor = new CANSparkMax(kLeftClimberMotorID, MotorType.kBrushless);
        mRightClimberMotor = new CANSparkMax(kRightClimberMotorID, MotorType.kBrushless);
    }

	public enum climberMotorToSpinEnum { right, left, both };

    @Override
    public void initialize() {
        
    }

    public void setClimberMotorSpeed(double speed, climberMotorToSpinEnum motorToSpin) {
        switch (motorToSpin) {
            case right:
                mRightClimberMotor.set(speed);
                break;
            case left:
                mLeftClimberMotor.set(speed);
                break;
            case both:
                mLeftClimberMotor.set(speed);
                mRightClimberMotor.set(speed);
                break;
        }
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
