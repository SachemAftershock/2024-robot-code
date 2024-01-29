package frc.robot.enums;

import edu.wpi.first.math.geometry.Pose2d;

public enum ClimberState {
    eUp(5),
    eDown(0);
    private double mClimberEncoderPosition;

    ClimberState(double mClimberEncoderPosition) {
        this.mClimberEncoderPosition = mClimberEncoderPosition;
    }
    public double getPosition(){
        return mClimberEncoderPosition;
    }
}
