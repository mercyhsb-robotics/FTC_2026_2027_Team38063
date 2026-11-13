package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Counts POLLEN launched by watching the flywheel speed dip each time a ball is pulled through it.
 *
 * <p>A dip only counts when the shooter was ready (at speed) beforehand and the feeder is running, so
 * spin-up and coast-down are never mistaken for shots. After a counted dip the counter is disarmed
 * until the flywheel is ready again and a short refractory time has passed, so one ball cannot be
 * counted twice. It never counts more than it is told is possible by its owner (the volley cap lives
 * in the controller).
 */
public final class ShotCounter {
    private final double dipFrac;
    private final double floorTps;
    private final double refractorySec;

    private int count;
    private boolean armed;
    private double lastShotSec = Double.NaN;
    private double startSec = Double.NaN;

    public ShotCounter(double dipFrac, double floorTps, double refractorySec) {
        this.dipFrac = dipFrac;
        this.floorTps = floorTps;
        this.refractorySec = refractorySec;
    }

    /** Starts a new volley: zero shots, disarmed until the shooter reports ready. */
    public void start(double nowSec) {
        count = 0;
        armed = false;
        lastShotSec = Double.NaN;
        startSec = nowSec;
    }

    /** Speed below which a feeding flywheel is considered to have just launched a ball. */
    public double dipThreshold(double targetTps) {
        return targetTps * (1.0 - dipFrac) - floorTps;
    }

    /**
     * Call once per loop.
     *
     * @param feeding  true while the feeder is pushing a ball
     * @param atSpeed  the debounced ready flag from {@link FlywheelReady}
     * @return true on the loop in which a new shot was counted
     */
    public boolean update(double nowSec, boolean feeding, boolean atSpeed, double targetTps, double measuredTps) {
        boolean refractoryOver = Double.isNaN(lastShotSec) || nowSec - lastShotSec >= refractorySec;
        if (!armed) {
            if (atSpeed && refractoryOver) {
                armed = true;
            }
            return false;
        }
        if (feeding && targetTps > 0.0 && measuredTps < dipThreshold(targetTps)) {
            count++;
            lastShotSec = nowSec;
            armed = false;
            return true;
        }
        return false;
    }

    public int count() {
        return count;
    }

    /** Seconds since the last counted shot, or since the volley started if there has been none. */
    public double secondsSinceLastShot(double nowSec) {
        double from = Double.isNaN(lastShotSec) ? startSec : lastShotSec;
        return Double.isNaN(from) ? 0.0 : nowSec - from;
    }

    public boolean armed() {
        return armed;
    }
}
