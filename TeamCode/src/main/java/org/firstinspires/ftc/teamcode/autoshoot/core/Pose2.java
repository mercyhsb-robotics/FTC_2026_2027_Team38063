package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * An immutable field pose: x and y in INCHES, heading in RADIANS (0 = +x, counter-clockwise
 * positive, stored in (-PI, PI]). The field frame is the Pedro frame used by this team: origin at
 * the corner where the audience wall meets the red wall, +x toward the blue side, +y away from the
 * audience.
 *
 * <p>A "pose" is just "where the robot is and which way it faces": a position plus a heading. "Immutable"
 * means a Pose2 never changes after it is made; to "change" one you build a new one (see
 * {@link #withHeading}). It is the plain-Java twin of Pedro's own {@code Pose}: {@code PedroDrive}
 * converts between the two so the rest of the logic never needs the Pedro library.
 *
 * <p>Robot-relative points use (forward, left): +forward is the direction the intake faces and
 * +left is 90 degrees counter-clockwise from it. The shooter and the Limelight face the BACK of
 * the robot, so they sit at negative forward. The robot frame's origin is the robot's rotation center
 * (the point it spins about), which is also the point this pose describes.
 *
 * <p>Example: a robot at pose (60, 20, heading 90 degrees) faces +y. A point 5 in behind it and 2 in to its
 * left is {@code fieldX(-5, 2)} = 58 and {@code fieldY(-5, 2)} = 15 in field coordinates.
 */
public final class Pose2 {
    /** Field x in inches (0 at the red wall, grows toward the blue wall). */
    public final double x;
    /** Field y in inches (0 at the audience wall, grows away from the audience). */
    public final double y;
    /** Heading in radians, counter-clockwise positive, 0 = facing +x, always wrapped into (-PI, PI]. */
    public final double heading;

    public Pose2(double x, double y, double headingRadians) {
        this.x = x;
        this.y = y;
        // Wrap on the way in so every Pose2 holds a heading in the same, comparable range.
        this.heading = Angles.normalize(headingRadians);
    }

    /** Builds a pose from a heading written in degrees (handy for constants such as the start pose). */
    public static Pose2 ofDegrees(double x, double y, double headingDegrees) {
        return new Pose2(x, y, Math.toRadians(headingDegrees));
    }

    /** The heading in degrees, in (-180, 180]. */
    public double headingDegrees() {
        return Math.toDegrees(heading);
    }

    /** A copy of this pose with the same position and a different heading (radians). */
    public Pose2 withHeading(double headingRadians) {
        return new Pose2(x, y, headingRadians);
    }

    /** Straight-line distance between the two positions, in inches (headings are ignored). */
    public double distanceTo(Pose2 other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    /** Straight-line distance from this position to the field point (px, py), in inches. */
    public double distanceTo(double px, double py) {
        return Math.hypot(px - x, py - y);
    }

    /**
     * Field x of a point given in this pose's robot frame. (A rotation by the heading, then a shift by this
     * pose's position: {@code forward} and {@code left} are in inches.)
     */
    public double fieldX(double forward, double left) {
        return x + forward * Math.cos(heading) - left * Math.sin(heading);
    }

    /** Field y of a point given in this pose's robot frame. */
    public double fieldY(double forward, double left) {
        return y + forward * Math.sin(heading) + left * Math.cos(heading);
    }

    /** Robot-frame forward component of a field point, relative to this pose. */
    public double forwardOf(double fieldX, double fieldY) {
        return (fieldX - x) * Math.cos(heading) + (fieldY - y) * Math.sin(heading);
    }

    /** Robot-frame left component of a field point, relative to this pose. */
    public double leftOf(double fieldX, double fieldY) {
        return -(fieldX - x) * Math.sin(heading) + (fieldY - y) * Math.cos(heading);
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.1f deg)", x, y, headingDegrees());
    }
}
