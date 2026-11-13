package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.Locale;

/** An immutable field point in inches (Pedro frame, see {@code Pose2}). Used for via points, targets and aim points. */
public final class Point2 {
    public final double x;
    public final double y;

    public Point2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distanceTo(Point2 other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    /** The image of this RED-frame point in the given alliance's frame (a half turn for BLUE). */
    public Point2 fromRed(Alliance alliance, double fieldSize) {
        return new Point2(alliance.fromRedX(x, fieldSize), alliance.fromRedY(y, fieldSize));
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Point2)) {
            return false;
        }
        Point2 p = (Point2) o;
        return Double.compare(x, p.x) == 0 && Double.compare(y, p.y) == 0;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(x, y);
    }

    @Override
    public String toString() {
        return String.format(Locale.ROOT, "(%.1f, %.1f)", x, y);
    }
}
