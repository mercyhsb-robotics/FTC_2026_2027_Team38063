package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Angle helpers for the auto-shoot logic. Everything in the core package uses RADIANS with a
 * counter-clockwise-positive, signed range of (-PI, PI]. Pedro Pathing reports headings in
 * [0, 2*PI) and the Limelight reports degrees; the hardware adapters convert once at the boundary.
 *
 * <p>Pure Java: no FTC SDK, Android or Pedro imports, so it can be unit tested on a PC.
 */
public final class Angles {
    public static final double TWO_PI = 2.0 * Math.PI;

    private Angles() {
    }

    /** Wraps an angle into (-PI, PI]. */
    public static double normalize(double radians) {
        double a = radians % TWO_PI;
        if (a <= -Math.PI) {
            a += TWO_PI;
        } else if (a > Math.PI) {
            a -= TWO_PI;
        }
        return a;
    }

    /**
     * The signed shortest rotation that takes {@code from} to {@code to}, in (-PI, PI].
     * Positive means counter-clockwise. Safe across the +/-PI seam: error(179 deg, -179 deg) = +2 deg.
     */
    public static double error(double from, double to) {
        return normalize(to - from);
    }

    /** Absolute value of {@link #error}. */
    public static double distance(double a, double b) {
        return Math.abs(error(a, b));
    }

    /** Rotates a heading by half a turn (used for the rear-facing shooter and the red/blue flip). */
    public static double opposite(double radians) {
        return normalize(radians + Math.PI);
    }
}
