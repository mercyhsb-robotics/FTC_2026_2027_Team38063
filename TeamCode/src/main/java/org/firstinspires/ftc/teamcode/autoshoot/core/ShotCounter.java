package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Counts POLLEN launched by watching the flywheel speed dip each time a ball is pulled through it.
 *
 * <p>A dip only counts when the shooter was ready (at speed) beforehand and the feeder is running, so
 * spin-up and coast-down are never mistaken for shots. After a counted dip the counter is disarmed
 * until the flywheel is ready again and a short refractory time has passed, so one ball cannot be
 * counted twice. It never counts more than it is told is possible by its owner (the volley cap lives
 * in the controller).
 *
 * <p>Why count by speed: the robot has no ball sensor. A ball squeezed through the spinning flywheel steals a
 * little of its energy, so the flywheel's speed (in encoder ticks per second, "tps") briefly drops. A drop below
 * {@link #dipThreshold} while the feeder runs is one shot. This is why {@code SHOT_DIP_FRAC} must be smaller than
 * the at-speed window ({@link FlywheelReady}): the dip must be noticed while the shooter still counts as ready.
 *
 * <p>"Refractory" means a short quiet period after an event during which a repeat is ignored. The counter is a
 * small state machine with two states: armed (waiting for a dip) and disarmed (waiting to recover from the last
 * one).
 *
 * <p>Callers: {@link AutoShootController} (once per loop while feeding) and the Shooter Tuner diagnostic, which
 * shows the dip so the team can pick {@code SHOT_DIP_FRAC}. Pure Java and clock-free: the time is passed in.
 */
public final class ShotCounter {
    private final double dipFrac;
    private final double floorTps;
    private final double refractorySec;

    // Shots counted since start().
    private int count;
    // True when the next dip will be counted as a shot.
    private boolean armed;
    // Time (seconds) of the last counted shot; NaN until there is one.
    private double lastShotSec = Double.NaN;
    // Time (seconds) when start() was called; NaN before the first start().
    private double startSec = Double.NaN;

    /**
     * @param dipFrac       how far below the target speed counts as a dip, as a fraction of the target (0.05 = 5 %)
     * @param floorTps      extra absolute slack in ticks per second (covers encoder noise); the speed must fall this
     *                      much further than {@code dipFrac} alone says
     * @param refractorySec seconds after a counted shot during which the counter stays disarmed
     */
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

    /** Speed below which a feeding flywheel is considered to have just launched a ball, in ticks per second. */
    public double dipThreshold(double targetTps) {
        return targetTps * (1.0 - dipFrac) - floorTps;
    }

    /**
     * Call once per loop.
     *
     * @param nowSec      the logic clock, in seconds
     * @param feeding     true while the feeder is pushing a ball
     * @param atSpeed     the debounced ready flag from {@link FlywheelReady}
     * @param targetTps   the commanded flywheel speed, in ticks per second
     * @param measuredTps the flywheel's measured speed, in ticks per second
     * @return true on the loop in which a new shot was counted
     */
    public boolean update(double nowSec, boolean feeding, boolean atSpeed, double targetTps, double measuredTps) {
        boolean refractoryOver = Double.isNaN(lastShotSec) || nowSec - lastShotSec >= refractorySec;
        // Disarmed: wait for the flywheel to recover (and the quiet period to pass) before watching for a dip again.
        if (!armed) {
            if (atSpeed && refractoryOver) {
                armed = true;
            }
            return false;
        }
        // Armed: a drop below the threshold while the feeder is pushing a ball means that ball just left.
        if (feeding && targetTps > 0.0 && measuredTps < dipThreshold(targetTps)) {
            count++;
            lastShotSec = nowSec;
            armed = false;
            return true;
        }
        return false;
    }

    /** Shots counted since {@link #start}. */
    public int count() {
        return count;
    }

    /**
     * Seconds since the last counted shot, or since the volley started if there has been none. The controller
     * uses this to decide the hopper is empty (the feeder runs but no ball has dipped the flywheel for a while).
     */
    public double secondsSinceLastShot(double nowSec) {
        double from = Double.isNaN(lastShotSec) ? startSec : lastShotSec;
        return Double.isNaN(from) ? 0.0 : nowSec - from;
    }

    /** True if the next dip would be counted (the flywheel has recovered since the last shot). */
    public boolean armed() {
        return armed;
    }
}
