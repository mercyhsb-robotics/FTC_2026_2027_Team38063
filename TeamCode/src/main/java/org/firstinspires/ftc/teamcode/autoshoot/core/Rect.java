package org.firstinspires.ftc.teamcode.autoshoot.core;

/** An immutable axis-aligned rectangle in field inches. Used for keep-out zones and field bounds. */
public final class Rect {
    public final double minX;
    public final double maxX;
    public final double minY;
    public final double maxY;

    public Rect(double x0, double x1, double y0, double y1) {
        this.minX = Math.min(x0, x1);
        this.maxX = Math.max(x0, x1);
        this.minY = Math.min(y0, y1);
        this.maxY = Math.max(y0, y1);
    }

    public static Rect centered(double cx, double cy, double width, double height) {
        return new Rect(cx - width / 2.0, cx + width / 2.0, cy - height / 2.0, cy + height / 2.0);
    }

    public Rect inflate(double margin) {
        return new Rect(minX - margin, maxX + margin, minY - margin, maxY + margin);
    }

    public boolean contains(double x, double y) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** True if the point is strictly inside (touching the boundary does not count). */
    public boolean containsStrictly(double x, double y) {
        return x > minX && x < maxX && y > minY && y < maxY;
    }

    public boolean overlaps(Rect o) {
        return minX < o.maxX && maxX > o.minX && minY < o.maxY && maxY > o.minY;
    }

    /** The same rectangle turned half a turn about (cx, cy). */
    public Rect rotated180(double cx, double cy) {
        return new Rect(2 * cx - maxX, 2 * cx - minX, 2 * cy - maxY, 2 * cy - minY);
    }

    /**
     * True if the segment from (x0,y0) to (x1,y1) passes through the INTERIOR of this rectangle
     * (Liang-Barsky clipping). Running exactly along an edge or touching a corner is not a hit.
     */
    public boolean segmentCrossesInterior(double x0, double y0, double x1, double y1) {
        double dx = x1 - x0;
        double dy = y1 - y0;
        double tMin = 0.0;
        double tMax = 1.0;
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
