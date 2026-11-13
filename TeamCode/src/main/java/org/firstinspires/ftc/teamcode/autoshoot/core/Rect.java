package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * An immutable axis-aligned rectangle in field inches: a box whose sides are parallel to the field's x and y axes
 * (programmers call this an AABB, "axis-aligned bounding box"). It is the only shape the planners need. The HIVE
 * frame, each FLOWER, the field boundary and the robot's footprint are all described as one of these, so
 * "is the robot on top of an obstacle?" becomes a simple comparison of four numbers.
 *
 * <p>Pure geometry with no robot knowledge. {@link FieldModel} creates the obstacle boxes,
 * {@link ShotGeometry#footprint} creates the robot's box, and {@link PathPlanner} and {@link ScoringPlanner}
 * test points, segments and other boxes against them. x and y are in the Pedro field frame (origin at the inner corner
 * where the audience wall meets the red wall, +x toward blue, +y away from the audience), all in inches.
 *
 * <p>Example: {@code new Rect(0, Field.SIZE_IN, 0, Field.SIZE_IN)} (141.348 in square) is the whole field, and {@code .inflate(-9)} on it gives the same
 * field with a 9 in strip removed along every wall (the strip the robot's center may not enter).
 */
public final class Rect {
    /** Smallest x of the box, in inches. */
    public final double minX;
    /** Largest x of the box, in inches. */
    public final double maxX;
    /** Smallest y of the box, in inches. */
    public final double minY;
    /** Largest y of the box, in inches. */
    public final double maxY;

    /**
     * Note the argument order: both x values first, then both y values (NOT x, y, width, height). Either value of
     * each pair may be the larger one; the constructor sorts them so {@code minX <= maxX} and {@code minY <= maxY}.
     */
    public Rect(double x0, double x1, double y0, double y1) {
        this.minX = Math.min(x0, x1);
        this.maxX = Math.max(x0, x1);
        this.minY = Math.min(y0, y1);
        this.maxY = Math.max(y0, y1);
    }

    /** A box around the center point (cx, cy): {@code width} is its size along x, {@code height} along y (inches). */
    public static Rect centered(double cx, double cy, double width, double height) {
        return new Rect(cx - width / 2.0, cx + width / 2.0, cy - height / 2.0, cy + height / 2.0);
    }

    /**
     * A copy grown by {@code margin} inches on every side. A negative margin shrinks it, which is how the planner
     * keeps the robot's center away from the walls (shrink the field) and away from obstacles (grow the obstacle).
     */
    public Rect inflate(double margin) {
        return new Rect(minX - margin, maxX + margin, minY - margin, maxY + margin);
    }

    /** True if the point is inside the box or exactly on its edge (the edge counts as inside here). */
    public boolean contains(double x, double y) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** True if the point is strictly inside (touching the boundary does not count). */
    public boolean containsStrictly(double x, double y) {
        return x > minX && x < maxX && y > minY && y < maxY;
    }

    /** True if the two boxes share some area. Boxes that only touch along an edge or corner do NOT overlap. */
    public boolean overlaps(Rect o) {
        return minX < o.maxX && maxX > o.minX && minY < o.maxY && maxY > o.minY;
    }

    /**
     * The same rectangle turned half a turn about (cx, cy). The RED and BLUE halves of the field are half-turn
     * images of each other (see {@link Alliance}). Currently nothing calls this; it is kept for tests and tools.
     */
    public Rect rotated180(double cx, double cy) {
        return new Rect(2 * cx - maxX, 2 * cx - minX, 2 * cy - maxY, 2 * cy - minY);
    }

    /**
     * True if the segment from (x0,y0) to (x1,y1) passes through the INTERIOR of this rectangle
     * (Liang-Barsky clipping). Running exactly along an edge or touching a corner is not a hit.
     *
     * <p>How it works: write the segment as a point that moves from t = 0 (start) to t = 1 (end). Each of the four
     * sides of the box limits t to the part where the point is still on the inside of that side. Keep the
     * overlap of all four limits (tMin..tMax). If anything is left, part of the segment is inside the box.
     * This is what lets the path planner ask "does this straight leg cut through the HIVE frame?" cheaply.
     */
    public boolean segmentCrossesInterior(double x0, double y0, double x1, double y1) {
        double dx = x1 - x0;
        double dy = y1 - y0;
        // tMin..tMax is the stretch of the segment (0 = start, 1 = end) still inside every side checked so far.
        double tMin = 0.0;
        double tMax = 1.0;
        // One entry per side: left, right, bottom, top. p is the segment's movement toward that side's outside
        // (positive = leaving through it, negative = entering through it, 0 = running parallel to it); q is how far
        // inside that side the start point is (negative = the start is already outside it).
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {x0 - minX, maxX - x0, y0 - minY, maxY - y0};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0.0) {
                if (q[i] <= 0.0) {
                    return false;       // parallel to this edge and not strictly inside its slab
                }
            } else {
                double t = q[i] / p[i];
                if (p[i] < 0.0) {
                    if (t > tMax) {
                        return false;
                    }
                    if (t > tMin) {
                        tMin = t;
                    }
                } else {
                    if (t < tMin) {
                        return false;
                    }
                    if (t < tMax) {
                        tMax = t;
                    }
                }
            }
        }
        // Reject grazing contact (a single point or an edge): need a segment of real length inside.
        return tMax - tMin > 1e-9;
    }

    @Override
    public String toString() {
        return String.format("x[%.2f,%.2f] y[%.2f,%.2f]", minX, maxX, minY, maxY);
    }
}
