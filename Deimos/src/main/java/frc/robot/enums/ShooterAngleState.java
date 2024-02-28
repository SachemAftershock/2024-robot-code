package frc.robot.enums;

import static frc.robot.Constants.ShooterConstants.*;
/**
 * 3 Angles. Positive direction is outwards from the robot, even though that means it rotates
 * towards the back.
 * The shooter is physically angled at 34 degrees relative to the ground -- we consider that as zero.
 * 
 * <p> eUnknown is defaulted at the value 0, but it is meant to serve as the "in-between" state, rather than actually
 * get used </p>
 */
public enum ShooterAngleState {
    eUnknown(0), // left at default 0
    eSpeaker(kSpeakerAngleSpeakerProfiler.getGoal()),
    // safe zone is when you touch the pillar at the center and try to shoot
    eSafeZone(kSpeakerAngleSafeZoneProfiler.getGoal()), // maybe 10 degrees or more lifted. TODO not used yet
    eAmp(kSpeakerAngleAmpProfiler.getGoal()), // should be 44-45 degrees
    eTrap(kSpeakerAngleTrapProfiler.getGoal()); // TODO wip
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
