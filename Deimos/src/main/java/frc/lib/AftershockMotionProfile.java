package frc.lib;

/**
 * Class to map position to a motor output based on a min and max range
 * 
 * @author Arhum Mudassir
 */

public class AftershockMotionProfile {

    private double minPosition;
    private double maxPosition;
    private double maxVelocity;
    private PositionVelocityProfile[] profiles;

    /**
     * Constructor for motion profiling class
     */
    public AftershockMotionProfile(double minPosition, double maxPosition, double maxVelocity, PositionVelocityProfile[] profiles) {
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.maxVelocity = maxVelocity;
        this.profiles = profiles;
    }

    /**
     * 
     * Finds the range in which the position is in and returns the corrosponding velocity
     * If the current position is not within range then velocity is 0;
     * If velocity output exceeds max velocity parameter, velocity is capped to maxVelocity
     * 
     * @param currentPosition
     * @return velocity output
     */
    public double getOutput(double currentPosition) {

        double velocity = 0.0;

        if(currentPosition >= maxPosition || currentPosition <= minPosition) {
            velocity = 0.0;
        }

        for(int i = 0; i < profiles.length; i++) {
            if(profiles[i].isInRange(currentPosition)) {
                velocity = profiles[i].getVelocity();
            }
        }

        if(velocity > maxVelocity) {
            velocity = maxVelocity;
        }

        return velocity;
    } 
}




