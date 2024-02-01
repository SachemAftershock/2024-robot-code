package frc.robot.enums;

import edu.wpi.first.math.geometry.Pose2d;

public enum ClimberState {
    eUp(5),
    eDown(2); //Holding position, must be at least 3 inches above bottoming out
    private double mClimberEncoderPosition;

    ClimberState(double mClimberEncoderPosition) {
        this.mClimberEncoderPosition = mClimberEncoderPosition;
    }
    public double getPosition(){
        return mClimberEncoderPosition;
    }
}
