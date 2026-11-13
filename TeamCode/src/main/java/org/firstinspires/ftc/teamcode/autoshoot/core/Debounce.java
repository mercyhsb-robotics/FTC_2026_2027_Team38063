package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * A two-edge time debouncer. The output goes true only after the raw input has been true for
 * {@code riseSec} without interruption, and false only after it has been false for {@code fallSec}.
 * Time is passed in (never read from a clock) so tests are deterministic.
 *
 * <p>"Debounce" comes from electronics: a real button's contacts chatter for a moment when pressed, so the
 * signal flickers on/off before it settles. A debouncer ignores flicker and only reports a change once the
 * new value has held steady. Here it is used on any noisy yes/no signal; {@code FlywheelReady} uses it so
 * "the flywheel is at speed" is not announced because the speed merely passed through the target window.
 *
 * <p>The two delays can differ on purpose: a long {@code riseSec} makes the output slow to become true (be
 * sure before saying yes), and a short {@code fallSec} makes it quick to become false (say no fast).
 *
 * <p>Example with riseSec = 0.10 and fallSec = 0.03: raw input true from t = 0.00 s on. {@code update} returns
 * false until t = 0.10 s, then true. If the raw input drops to false at t = 1.00 s, the output stays true until
 * t = 1.03 s. If the raw input is false for only 0.01 s and comes back, the output never changes at all.
 */
public final class Debounce {
    private final double riseSec;
    private final double fallSec;
    // The debounced answer that callers see.
    private boolean output;
    // True while the raw input disagrees with the output and we are waiting to see if it stays that way.
    private boolean pending;
    // The time at which the raw input first disagreed with the output (valid while pending is true).
    private double pendingSince;

    /** Both delays are in seconds; a negative value is treated as 0 (no delay on that edge). */
    public Debounce(double riseSec, double fallSec) {
        this.riseSec = Math.max(0.0, riseSec);
        this.fallSec = Math.max(0.0, fallSec);
    }

    /**
     * Feeds in the latest raw value and returns the debounced output. Call once per loop.
     *
     * @param raw    the noisy input right now
     * @param nowSec the current time in seconds (any clock, as long as it only moves forward)
     */
    public boolean update(boolean raw, double nowSec) {
        // Raw agrees with the output: nothing to decide, and any half-finished wait is cancelled
        // (that is what makes a brief flicker harmless).
        if (raw == output) {
            pending = false;
            return output;
        }
        // Raw just started to disagree: note when, so we can time how long it stays that way.
        if (!pending) {
            pending = true;
            pendingSince = nowSec;
        }
        // Going true waits riseSec; going false waits fallSec.
        double needed = raw ? riseSec : fallSec;
        if (nowSec - pendingSince >= needed) {
            output = raw;
            pending = false;
        }
        return output;
    }

    /** The output as of the last {@link #update} (false before the first call). */
    public boolean get() {
        return output;
    }

    /** Forgets everything: the output goes back to false and any wait in progress is cancelled. */
    public void reset() {
        output = false;
        pending = false;
    }
}
