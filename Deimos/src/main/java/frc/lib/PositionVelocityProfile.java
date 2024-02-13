package frc.lib;

/**
 * Class to hold position range objects for user defined ranges
 * 
 * @author Arhum Mudassir
 */
public class PositionVelocityProfile {

    private double min;
    private double max;
    private double velocity;

    public PositionVelocityProfile(double min, double max, double velocity) {
        this.min = min;
        this.max = max;
        this.velocity = velocity;
    }

    public double getMin() {
        return min;
    }

    public double getMax() {
        return max;
    }

    public double getVelocity() {
        return velocity;
    }

    public boolean isInRange(double val) {

        if(val <= max && val >= min) {
            return true;
        }

        return false;
    }
}
