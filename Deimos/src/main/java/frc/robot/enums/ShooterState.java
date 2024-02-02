package frc.robot.enums;

public enum ShooterState {
    eSpeaker(0),
    eSafeZone(0),
    eAmp(0);
    // 
    private double angle;
    
    ShooterState(double angle) {
        this.angle = angle;
    }
    public double getAngle() {
        return angle;
    }
}
