package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Decides whether the flywheel is "at speed": its measured velocity is inside a window around the
 * commanded target for long enough. The rising edge is slow (so a speed that is only passing through
 * the window during spin-up does not count) and the falling edge is fast (so a ball that dragged the
 * speed down is noticed quickly and the next ball waits).
 */
public final class FlywheelReady {
    private final double belowFrac;
    private final double aboveFrac;
    private final double floorTps;
    private final Debounce debounce;

    public FlywheelReady(double belowFrac, double aboveFrac, double floorTps, double riseSec, double fallSec) {
        this.belowFrac = belowFrac;
        this.aboveFrac = aboveFrac;
        this.floorTps = floorTps;
        this.debounce = new Debounce(riseSec, fallSec);
    }

    /** Lowest speed that still counts as inside the window for this target. */
    public double lowerBound(double targetTps) {
        return targetTps * (1.0 - belowFrac) - floorTps;
    }

    /** Highest speed that still counts as inside the window for this target. */
    public double upperBound(double targetTps) {
        return targetTps * (1.0 + aboveFrac) + floorTps;
    }

    /** True if the speed is inside the window right now, ignoring the debounce. */
    public boolean rawInWindow(double targetTps, double measuredTps) {
        return targetTps > 0.0 && measuredTps >= lowerBound(targetTps) && measuredTps <= upperBound(targetTps);
    }

    /** Call once per loop. {@code running} is false when the flywheel is not being asked to spin. */
    public boolean update(double nowSec, boolean running, double targetTps, double measuredTps) {
        boolean raw = running && rawInWindow(targetTps, measuredTps);
        return debounce.update(raw, nowSec);
    }

    public boolean isAtSpeed() {
        return debounce.get();
    }

    public void reset() {
        debounce.reset();
    }
}
