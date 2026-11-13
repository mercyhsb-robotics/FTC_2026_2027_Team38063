package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Angle helpers for the auto-shoot logic. Everything in the core package uses RADIANS with a
 * counter-clockwise-positive, signed range of (-PI, PI]. Pedro Pathing reports headings in
 * [0, 2*PI) and the Limelight reports degrees; the hardware adapters convert once at the boundary.
 *
 * <p><b>Why this class exists: heading wrap-around.</b> A heading is a direction, so it goes round in a
 * circle: 359 degrees and 1 degree are only 2 degrees apart, but plain subtraction says 358. If the robot
 * code subtracted headings directly it would spin the long way round, or think it was badly off target when
 * it was nearly on it. Every heading difference in the project goes through {@link #error} instead.
 * Example: {@code Angles.error(Math.toRadians(179), Math.toRadians(-179))} is about +0.035 rad (+2 degrees).
 *
 * <p>Used by {@link Pose2}, {@link Alliance}, the camera and cluster maths, the controller and
 * {@code PedroDrive}.
 *
 * <p>Pure Java: no FTC SDK, Android or Pedro imports, so it can be unit tested on a PC.
 */
public final class Angles {
    /** A full turn, in radians (360 degrees). */
    public static final double TWO_PI = 2.0 * Math.PI;

    private Angles() {
    }

    /** Wraps an angle into (-PI, PI]. */
    public static double normalize(double radians) {
        // Java's % keeps the sign of the input, so the result is in (-2*PI, 2*PI); the two checks below
        // fold the rest into (-PI, PI].
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

    /** Absolute value of {@link #error}: how far apart two headings are, ignoring direction (0 to PI radians). */
    public static double distance(double a, double b) {
        return Math.abs(error(a, b));
    }

    /** Rotates a heading by half a turn (used for the rear-facing shooter and the red/blue flip). */
    public static double opposite(double radians) {
        return normalize(radians + Math.PI);
    }
}
