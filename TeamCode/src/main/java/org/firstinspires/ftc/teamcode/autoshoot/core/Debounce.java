package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * A two-edge time debouncer. The output goes true only after the raw input has been true for
 * {@code riseSec} without interruption, and false only after it has been false for {@code fallSec}.
 * Time is passed in (never read from a clock) so tests are deterministic.
 */
public final class Debounce {
    private final double riseSec;
    private final double fallSec;
    private boolean output;
    private boolean pending;
    private double pendingSince;

    public Debounce(double riseSec, double fallSec) {
        this.riseSec = Math.max(0.0, riseSec);
        this.fallSec = Math.max(0.0, fallSec);
    }

    public boolean update(boolean raw, double nowSec) {
        if (raw == output) {
            pending = false;
            return output;
        }
        if (!pending) {
            pending = true;
            pendingSince = nowSec;
        }
        double needed = raw ? riseSec : fallSec;
        if (nowSec - pendingSince >= needed) {
            output = raw;
            pending = false;
        }
        return output;
    }

    public boolean get() {
        return output;
    }

    public void reset() {
        output = false;
        pending = false;
    }
}
