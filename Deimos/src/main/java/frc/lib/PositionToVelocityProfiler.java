package frc.lib;

import java.util.ArrayList;

/**
 * Maps a <b>given position range [a,b) </b> to a <b>velocity</b>, of arbitrary units.
 * <p>
 * Instead of using a PID, we just take an input of position, find out what two
 * positions we're in between in our lookup table,
 * find the velocity we want when we're within said interval (according to our profiles), then return that
 * velocity. Essentially: glorified `if` statements.
 * </p>
 * 
 * <pre>
 * <code>
 * // Making a profile
 * public static final PositionToVelocityProfiler motionProfile = (new PositionToVelocityProfiler(4.0))
 *  .setGoal(4.0) // Has no effect. Is only used to communicate to enums.
 *  .addProfileEntry(0.0, 3.5, 5.0)
 *  .addProfileEntry(3.5, 4.5, 0) // our declared goal is 4, so we need to honor that and stop at 0.
 *  .addProfileEntry(lowerBound position, higherBound position, desiredVelocity);
 * </pre>
 */
public class PositionToVelocityProfiler {
    private ArrayList<Double[]> intervalActionsList;

    private Double positionGoal;
    private boolean positionGoalInitialized = false;

    public PositionToVelocityProfiler() {
        intervalActionsList = new ArrayList<Double[]>();
    }

    /**
     * Gets the goal position as declared by the profiler. Useful for enums
     * @return the goal position, usually where we manually set the speed as ~0.
     */
    public double getGoal() {
        return positionGoal;
    }

    /**
     * If this profiler is linked to an enum, it will be useful to only need to edit this profiler to
     * edit the enum. Setting the goal has no actual purpose other than readability -- you will need to
     * implement that yourself in the lookup tables.
     * @param positionGoal
     * @return this (for method chaining)
     */
    public PositionToVelocityProfiler setGoal(double positionGoal) {
        if (this.positionGoal != null) {
            System.out.println("Error: incorrectly tried to reinitialize position goal");
        } else {
            this.positionGoal = positionGoal;
        }
        return this;
    }

    /**
     * <p>
     * When we call <code>calculate(double input)</code>, if there is an added profile entry where the input
     * is in the specified interval [a,b), the associated velocity will be returned. Otherwise, returns 0.
     * </p>
     * 
     * 
     * <pre>
     * <code>
     * // Example of use
     * public static final PositionToVelocityProfiler motionProfile = (new PositionToVelocityProfiler(4.0))
     *  .addProfileEntry(0.0, 3.5, 5.0)
     *  .addProfileEntry(3.5, 4.5, 0) // our declared goal is 4, so we stop at 0
     *  .addProfileEntry(lowerBound position, higherBound position, desiredVelocity);
     * 
     * motionProfile.calculate(3.3); // returns 5.0
     * </pre>
     * 
     * @param lowerBound [a
     * @param higherBound b)
     * @param desiredVelocity associated velocity
     */
    public PositionToVelocityProfiler addProfileEntry(double lowerBound, double higherBound, double desiredVelocity) {
        this.intervalActionsList.add(new Double[] {
                lowerBound, higherBound, desiredVelocity
        });
        return this;
    }

    /**
     * Takes an input position, and uses the saved arrays of lowerBounds,
     * higherBounds, and desired velocities to return our wanted velocity
     * 
     * <p>
     * For all of our profiles, if input position is in [lowerBound, higherBound), return
     * the associated velocity.
     * </p>
     * 
     * <p> <i>
     * Returns 0 if the inputPosition is not found in any of our profile intervals.
     * </i> </p>
     * 
     * @param inputPosition arbitrary units
     * @return desired velocity of arbitrary units based on the profiles (intervals
     *         and desired velocities) you've added via <code>addEntry</code>
     */
    public double calculate(double inputPosition) {
        for (Double[] profile : intervalActionsList) {
            double low = profile[0];
            double hi = profile[1];
            double desiredVelocity = profile[2];

            if (low <= inputPosition && inputPosition < hi) {
                return desiredVelocity;
            }
        }
        return 0; // Not found at all.
    }

    /**
     * calculate with print statements
     * @param inputPosition
     * @return
     */
    public double calculateDebug(double inputPosition) {
        for (Double[] profile : intervalActionsList) {
            double low = profile[0];
            double hi = profile[1];
            double desiredVelocity = profile[2];

            if (low <= inputPosition && inputPosition < hi) {
                System.out.println("Mapped pos=" + inputPosition + " to v=" + desiredVelocity);
                return desiredVelocity;
            }
        }
        System.out.println("Didn't find inputPosition "+inputPosition);
        return 0; // Not found at all.
    }
}
