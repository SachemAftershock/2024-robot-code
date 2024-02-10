package frc.robot.enums;

/**
 * 3 Angles. Positive direction is inward towards the robot, negative direction is away.
 * The shooter is physically angled at 34 degrees relative to the ground (just look at the
 * smaller angle) --- we consider that as zero.
 */
public enum ShooterAngleState {
    eSpeaker(-20),
    // safe zone is when you touch the pillar at the center and try to shoot
    eSafeZone(0), // maybe 10 degreesa bit lifted
    eAmp(-45); // should be 44-45 degrees
    // 
    private double angle;
    
    ShooterAngleState(double angle) {
        this.angle = angle;
    }
    /**
     * 
     * @return degrees
     */
    public double getAngle() {
        return angle;
    }
}
