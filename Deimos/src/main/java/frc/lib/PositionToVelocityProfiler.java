package frc.lib;

import java.util.ArrayList;

/**
 * Maps a <b>given position range [a,b) </b> to a <b>velocity</b>, of arbitrary units.
 * <p>
 * Instead of using a PID, we just take an input of position, find out what two
 * positions we're in between,
 * find the velocity we want when we're in those positions (according to our profiles), and return that
 * velocity. Essentially: glorified `if` statements.
 * </p>
 * 
 * <pre>
 * <code>
 * // Making a profile
 * public static final PositionToVelocityProfiler motionProfile = (new PositionToVelocityProfiler())
 *  .addProfileEntry(0, 3, 5)
 *  .addProfileEntry(lowerBound position, higherBound position, desiredVelocity);
 * </pre>
 */
public class PositionToVelocityProfiler {
    private ArrayList<Double[]> intervalActionsList;

    public PositionToVelocityProfiler() {
        intervalActionsList = new ArrayList<Double[]>();
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
     * // example of use
     * public static final PositionToVelocityProfiler motionProfile = (new PositionToVelocityProfiler())
     *  .addProfileEntry(0, 3, 5)
     *  .addProfileEntry(lowerBound position, higherBound position, desiredVelocity);
     * </code>
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
