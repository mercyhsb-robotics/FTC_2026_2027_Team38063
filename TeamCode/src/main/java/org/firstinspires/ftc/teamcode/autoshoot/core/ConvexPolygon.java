package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * An immutable CONVEX polygon on the floor, in field inches: a flat shape with no dents (every corner turns the same way), like a
 * rectangle, a trapezoid or a hexagon. It is the shape the exact HIVE solid is made of (a foot bar, the slanted stub of a leg), and it
 * is what a robot's rectangle is compared with when a plain {@link Rect} box is not accurate enough.
 *
 * <p>Why convex only: two convex shapes are apart exactly when some straight line has one on each side of it, and that line can always
 * be chosen along one of the polygons' edges (the "separating axis" idea; programmers call it SAT). That makes the distance between two of
 * them a few lines of code with nothing to loop over but their edges. A shape with a dent (the whole HIVE footprint, with the lane between
 * its feet) is a LIST of convex polygons, see {@link HiveSolid}.
 *
 * <p>The corners are kept counter-clockwise. Touching is not overlapping: the gap of two shapes that only share a boundary is 0.
 *
 * <p>Pure geometry with no robot knowledge, in the same field frame as {@link Rect}. {@link Footprint#signedGap(ConvexPolygon)} compares the
 * robot's rectangle with one, and {@link RouteSweep} uses that gap to prove a drive never touches a leg or a foot bar.
 */
public final class ConvexPolygon {
    /** Two corners closer than this (inches) are the same corner. It only absorbs rounding. */
    private static final double SAME_CORNER_IN = 1e-9;
    /** A corner that bends the wrong way by less than this (square inches of area) is rounding, not a dent. */
    private static final double DENT_TOLERANCE = 1e-9;

    private final double[] xs;
    private final double[] ys;

    private ConvexPolygon(double[] xs, double[] ys) {
        this.xs = xs;
        this.ys = ys;
    }

    // ---- making polygons -------------------------------------------------------------------------------------------

    /**
     * A polygon from its corners given as x0, y0, x1, y1, ... in either turning direction. Throws IllegalArgumentException if there are fewer
     * than 3 corners, if the area is zero or if the corners make a dent (the polygon is not convex).
     */
    public static ConvexPolygon of(double... xy) {
        if (xy.length % 2 != 0) {
            throw new IllegalArgumentException("a polygon needs x and y for every corner, got " + xy.length + " numbers");
        }
        List<Point2> corners = new ArrayList<Point2>();
        for (int i = 0; i < xy.length; i += 2) {
            Point2 p = new Point2(xy[i], xy[i + 1]);
            if (corners.isEmpty() || corners.get(corners.size() - 1).distanceTo(p) > SAME_CORNER_IN) {
                corners.add(p);
            }
        }
        if (corners.size() > 1 && corners.get(0).distanceTo(corners.get(corners.size() - 1)) <= SAME_CORNER_IN) {
            corners.remove(corners.size() - 1);
        }
        return fromCorners(corners);
    }

    /** The box as a polygon (corners counter-clockwise from the lower left). */
    public static ConvexPolygon of(Rect r) {
        return new ConvexPolygon(new double[] {r.minX, r.maxX, r.maxX, r.minX}, new double[] {r.minY, r.minY, r.maxY, r.maxY});
    }

    /**
     * The smallest convex polygon that holds all the points (their "convex hull": picture a rubber band stretched around nails at the
     * points). Throws IllegalArgumentException if the points do not span an area.
     */
    public static ConvexPolygon hull(List<Point2> points) {
        List<Point2> sorted = new ArrayList<Point2>(points);
        Collections.sort(sorted, new Comparator<Point2>() {
            @Override
            public int compare(Point2 a, Point2 b) {
                return a.x != b.x ? Double.compare(a.x, b.x) : Double.compare(a.y, b.y);
            }
        });
        if (sorted.size() < 3) {
            throw new IllegalArgumentException("a polygon needs at least 3 points, got " + sorted.size());
        }
        // Andrew's monotone chain: build the lower side left to right, then the upper side right to left.
        List<Point2> chain = new ArrayList<Point2>();
        for (int pass = 0; pass < 2; pass++) {
            int start = chain.size();
            List<Point2> run = pass == 0 ? sorted : reversed(sorted);
            for (Point2 p : run) {
                while (chain.size() >= start + 2 && cross(chain.get(chain.size() - 2), chain.get(chain.size() - 1), p) <= 0.0) {
                    chain.remove(chain.size() - 1);
                }
                chain.add(p);
            }
            chain.remove(chain.size() - 1);       // the last point of each side is the first point of the next
        }
        return fromCorners(chain);
    }

    private static List<Point2> reversed(List<Point2> list) {
        List<Point2> out = new ArrayList<Point2>(list);
        Collections.reverse(out);
        return out;
    }

    private static double cross(Point2 o, Point2 a, Point2 b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    }

    private static ConvexPolygon fromCorners(List<Point2> corners) {
        int n = corners.size();
        if (n < 3) {
            throw new IllegalArgumentException("a polygon needs at least 3 different corners, got " + n);
        }
        double area2 = 0.0;
        for (int i = 0; i < n; i++) {
            Point2 a = corners.get(i);
            Point2 b = corners.get((i + 1) % n);
            area2 += a.x * b.y - b.x * a.y;
        }
        if (Math.abs(area2) <= DENT_TOLERANCE) {
            throw new IllegalArgumentException("the corners enclose no area");
        }
        if (area2 < 0.0) {
            corners = reversed(corners);                 // clockwise: turn it around so that it runs counter-clockwise
        }
        double[] xs = new double[n];
        double[] ys = new double[n];
        for (int i = 0; i < n; i++) {
            xs[i] = corners.get(i).x;
            ys[i] = corners.get(i).y;
        }
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            int k = (i + 2) % n;
            double turn = (xs[j] - xs[i]) * (ys[k] - ys[j]) - (ys[j] - ys[i]) * (xs[k] - xs[j]);
            if (turn < -DENT_TOLERANCE) {
                throw new IllegalArgumentException("the polygon has a dent at corner " + j + ": it is not convex");
            }
        }
        return new ConvexPolygon(xs, ys);
    }

    // ---- reading it ------------------------------------------------------------------------------------------------

    /** Number of corners. */
    public int size() {
        return xs.length;
    }

    /** Corner {@code i}, counter-clockwise. */
    public Point2 corner(int i) {
        return new Point2(xs[i], ys[i]);
    }

    /** The smallest axis-aligned box that holds the polygon. */
    public Rect bounds() {
        double minX = xs[0];
        double maxX = xs[0];
        double minY = ys[0];
        double maxY = ys[0];
        for (int i = 1; i < xs.length; i++) {
            minX = Math.min(minX, xs[i]);
            maxX = Math.max(maxX, xs[i]);
            minY = Math.min(minY, ys[i]);
            maxY = Math.max(maxY, ys[i]);
        }
        return new Rect(minX, maxX, minY, maxY);
    }

    /** The area in square inches. */
    public double area() {
        double sum = 0.0;
        for (int i = 0; i < xs.length; i++) {
            int j = (i + 1) % xs.length;
            sum += xs[i] * ys[j] - xs[j] * ys[i];
        }
        return sum / 2.0;
    }

    /** True if the point is inside or on the boundary. */
    public boolean contains(double x, double y) {
        for (int i = 0; i < xs.length; i++) {
            int j = (i + 1) % xs.length;
            // for a counter-clockwise polygon the inside is on the left of every edge
            if ((xs[j] - xs[i]) * (y - ys[i]) - (ys[j] - ys[i]) * (x - xs[i]) < -SAME_CORNER_IN) {
                return false;
            }
        }
        return true;
    }

    /**
     * The x range the polygon covers on the horizontal line at height {@code y}, as {min, max}, or null if the line misses it. It is
     * how wide a corridor the polygon leaves open on one side at that row.
     */
    public double[] xRangeAt(double y) {
        double lo = Double.POSITIVE_INFINITY;
        double hi = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < xs.length; i++) {
            int j = (i + 1) % xs.length;
            double y0 = ys[i];
            double y1 = ys[j];
            if (y < Math.min(y0, y1) - SAME_CORNER_IN || y > Math.max(y0, y1) + SAME_CORNER_IN) {
                continue;
            }
            double x;
            if (Math.abs(y1 - y0) <= SAME_CORNER_IN) {
                lo = Math.min(lo, Math.min(xs[i], xs[j]));
                hi = Math.max(hi, Math.max(xs[i], xs[j]));
                continue;
            }
            x = xs[i] + (xs[j] - xs[i]) * (y - y0) / (y1 - y0);
            lo = Math.min(lo, x);
            hi = Math.max(hi, x);
        }
        return lo > hi ? null : new double[] {lo, hi};
    }

    /** The same polygon turned half a turn about (cx, cy), the way the other alliance sees it. */
    public ConvexPolygon rotated180(double cx, double cy) {
        double[] nx = new double[xs.length];
        double[] ny = new double[xs.length];
        for (int i = 0; i < xs.length; i++) {
            nx[i] = 2.0 * cx - xs[i];
            ny[i] = 2.0 * cy - ys[i];
        }
        return new ConvexPolygon(nx, ny);              // a half turn keeps the corners counter-clockwise
    }

    // ---- distance --------------------------------------------------------------------------------------------------

    /**
     * The signed gap between this polygon and another, in inches: POSITIVE = they are that far apart (the shortest distance between the two
     * shapes), 0 = they just touch, NEGATIVE = they overlap and this is how deep (the shortest distance one would have to slide to come
     * free). It is a true distance, so if every point of one shape moves by at most {@code m} inches the gap changes by at most {@code m}: the
     * property that lets {@code RouteSweep} and the route validator prove that nothing slips between two samples of a drive.
     *
     * <p>How: first the separating-axis test. For every edge of both polygons, project both shapes on the edge's normal (the direction
     * straight out of the edge) and measure the space between the two shadows; the largest such space is negative exactly when the shapes
     * overlap, and then it is minus the overlap depth. When they are apart the true distance is between a corner of one and an edge of the other,
     * so those are measured one by one.
     */
    public double signedGap(ConvexPolygon other) {
        double best = Math.max(separation(this, other), separation(other, this));
        if (best <= 0.0) {
            return best;
        }
        double nearest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < xs.length; i++) {
            nearest = Math.min(nearest, other.distanceToBoundary(xs[i], ys[i]));
        }
        for (int i = 0; i < other.xs.length; i++) {
            nearest = Math.min(nearest, distanceToBoundary(other.xs[i], other.ys[i]));
        }
        return nearest;
    }

    /** The largest space between the shadows of {@code a} and {@code b}, over the normals of the edges of {@code a}. */
    private static double separation(ConvexPolygon a, ConvexPolygon b) {
        double best = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < a.xs.length; i++) {
            int j = (i + 1) % a.xs.length;
            // the outward normal of a counter-clockwise edge points to its right
            double nx = a.ys[j] - a.ys[i];
            double ny = -(a.xs[j] - a.xs[i]);
            double length = Math.hypot(nx, ny);
            nx /= length;
            ny /= length;
            double aMax = Double.NEGATIVE_INFINITY;
            for (int k = 0; k < a.xs.length; k++) {
                aMax = Math.max(aMax, a.xs[k] * nx + a.ys[k] * ny);
            }
            double bMin = Double.POSITIVE_INFINITY;
            for (int k = 0; k < b.xs.length; k++) {
                bMin = Math.min(bMin, b.xs[k] * nx + b.ys[k] * ny);
            }
            best = Math.max(best, bMin - aMax);
        }
        return best;
    }

    /** The distance from a point to the nearest point of this polygon's boundary (0 on it). */
    private double distanceToBoundary(double px, double py) {
        double nearest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < xs.length; i++) {
            int j = (i + 1) % xs.length;
            double dx = xs[j] - xs[i];
            double dy = ys[j] - ys[i];
            double t = ((px - xs[i]) * dx + (py - ys[i]) * dy) / (dx * dx + dy * dy);
            t = Math.max(0.0, Math.min(1.0, t));
            nearest = Math.min(nearest, Math.hypot(px - (xs[i] + t * dx), py - (ys[i] + t * dy)));
        }
        return nearest;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("polygon[");
        for (int i = 0; i < xs.length; i++) {
            sb.append(i == 0 ? "" : " ").append(String.format(Locale.ROOT, "(%.2f,%.2f)", xs[i], ys[i]));
        }
        return sb.append(']').toString();
    }
}
