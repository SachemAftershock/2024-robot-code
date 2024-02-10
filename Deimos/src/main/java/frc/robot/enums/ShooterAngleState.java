package frc.robot.enums;

public enum ShooterAngleState {
    eSpeaker(0),
    eSafeZone(0),
    eAmp(0);
    // 
    private double angle;
    
    ShooterAngleState(double angle) {
        this.angle = angle;
    }
    public double getAngle() {
        return angle;
    }
}
