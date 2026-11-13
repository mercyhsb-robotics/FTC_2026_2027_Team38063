package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Decides whether the flywheel is "at speed": its measured velocity is inside a window around the
 * commanded target for long enough. The rising edge is slow (so a speed that is only passing through
 * the window during spin-up does not count) and the falling edge is fast (so a ball that dragged the
 * speed down is noticed quickly and the next ball waits).
 *
 * <p>Why it exists: a ball fed into a flywheel that is still spinning up leaves too slowly and misses the cell.
 * So the feeder may only run while this says "ready". Speeds are in encoder ticks per second ("tps").
 *
 * <p>Two ideas, both simple. The WINDOW is the target speed plus or minus a few percent (plus a small fixed
 * slack in tps for encoder noise): inside it the speed is close enough. The DEBOUNCE ({@link Debounce}) stops
 * the answer flickering: "ready" is only announced after the speed has stayed inside for {@code riseSec}, and
 * "not ready" only after it has stayed outside for {@code fallSec}.
 *
 * <p>Example (the default constants at the time of writing): target 1200 tps, window -6 % / +6 % with 20 tps
 * slack gives 1108 .. 1292 tps. The speed enters the window at t = 0; "ready" turns on at t = 0.10 s if it
 * never left.
 *
 * <p>Callers: {@link AutoShootController} while feeding (one instance) and the Shooter Tuner diagnostic.
 * Pure Java and clock-free: the caller passes the time in.
 */
public final class FlywheelReady {
    private final double belowFrac;
    private final double aboveFrac;
    private final double floorTps;
    private final Debounce debounce;

    /**
     * @param belowFrac how far BELOW the target still counts as ready, as a fraction of the target (0.06 = 6 %)
     * @param aboveFrac how far ABOVE the target still counts as ready, as a fraction of the target
     * @param floorTps  absolute slack in ticks per second added to both ends of the window
     * @param riseSec   seconds the speed must stay inside the window before "ready" is announced
     * @param fallSec   seconds the speed must stay outside before "not ready" is announced (keep short)
     */
    public FlywheelReady(double belowFrac, double aboveFrac, double floorTps, double riseSec, double fallSec) {
        this.belowFrac = belowFrac;
        this.aboveFrac = aboveFrac;
        this.floorTps = floorTps;
        this.debounce = new Debounce(riseSec, fallSec);
    }

    /** Lowest speed that still counts as inside the window for this target, in ticks per second. */
    public double lowerBound(double targetTps) {
        return targetTps * (1.0 - belowFrac) - floorTps;
    }

    /** Highest speed that still counts as inside the window for this target, in ticks per second. */
    public double upperBound(double targetTps) {
        return targetTps * (1.0 + aboveFrac) + floorTps;
    }

    /** True if the speed is inside the window right now, ignoring the debounce. */
    public boolean rawInWindow(double targetTps, double measuredTps) {
        return targetTps > 0.0 && measuredTps >= lowerBound(targetTps) && measuredTps <= upperBound(targetTps);
    }

    /**
     * Call once per loop and use the returned (debounced) answer.
     *
     * @param nowSec      the logic clock, in seconds
     * @param running     false when the flywheel is not being asked to spin (then the answer heads to "not ready")
     * @param targetTps   the commanded speed, in ticks per second
     * @param measuredTps the flywheel's measured speed, in ticks per second
     * @return true if the flywheel is at speed
     */
    public boolean update(double nowSec, boolean running, double targetTps, double measuredTps) {
        boolean raw = running && rawInWindow(targetTps, measuredTps);
        return debounce.update(raw, nowSec);
    }

    /** The answer from the most recent {@link #update} (it does not recompute anything). */
    public boolean isAtSpeed() {
        return debounce.get();
    }

    /** Forgets everything: "not ready" until the speed has been inside the window for the rise time again. */
    public void reset() {
        debounce.reset();
    }
}
