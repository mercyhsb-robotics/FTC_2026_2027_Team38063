package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * An immutable field pose: x and y in INCHES, heading in RADIANS (0 = +x, counter-clockwise
 * positive, stored in (-PI, PI]). The field frame is the Pedro frame used by this team: origin at
 * the corner where the audience wall meets the red wall, +x toward the blue side, +y away from the
 * audience.
 *
 * <p>Robot-relative points use (forward, left): +forward is the direction the intake faces and
 * +left is 90 degrees counter-clockwise from it. The shooter and the Limelight face the BACK of
 * the robot, so they sit at negative forward.
 */
public final class Pose2 {
    public final double x;
    public final double y;
    public final double heading;

    public Pose2(double x, double y, double headingRadians) {
        this.x = x;
        this.y = y;
        this.heading = Angles.normalize(headingRadians);
    }

    public static Pose2 ofDegrees(double x, double y, double headingDegrees) {
        return new Pose2(x, y, Math.toRadians(headingDegrees));
    }

    public double headingDegrees() {
        return Math.toDegrees(heading);
    }

    public Pose2 withHeading(double headingRadians) {
        return new Pose2(x, y, headingRadians);
    }

    public double distanceTo(Pose2 other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    public double distanceTo(double px, double py) {
        return Math.hypot(px - x, py - y);
    }

    /** Field x of a point given in this pose's robot frame. */
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
