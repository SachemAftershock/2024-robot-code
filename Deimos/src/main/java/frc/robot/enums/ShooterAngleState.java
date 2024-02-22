package frc.robot.enums;

/**
 * 3 Angles. Positive direction is outwards from the robot, even though that means it rotates
 * towards the back.
 * The shooter is physically angled at 34 degrees relative to the ground (just look at the
 * smaller angle) --- we consider that as zero.
 * 
 * <p> eUnknown is defaulted at the value 0, but it is meant to serve as the "in-between" state, rather than actually
 * get used </p>
 */
public enum ShooterAngleState {
    eUnknown(0), // left at default 0
    eSpeaker(0),
    // safe zone is when you touch the pillar at the center and try to shoot
    eSafeZone(0), // maybe 10 degreesa bit lifted
    eAmp(95); // should be 44-45 degrees
    /* NOTE: you must also change the velocity profiler constant in Constants.java for shooter angle
     */ 



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
