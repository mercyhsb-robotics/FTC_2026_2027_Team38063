package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The robot's floor footprint at a pose: a rectangle {@code LENGTH x WIDTH} (front to back by side to
 * side) rotated to the pose's heading about the robot's rotation center. The corners run
 * counter-clockwise starting at the front-left one.
 *
 * <p>All overlap tests treat touching as NOT overlapping (a robot may stand against a wall or a
 * keep-out edge); a penetration of more than a nanoinch counts.
 */
public final class Footprint {
    private static final double EPS = 1e-9;

    private final Pose2 pose;
    private final double halfLength;
    private final double halfWidth;
    private final double[] xs = new double[4];
    private final double[] ys = new double[4];

    private Footprint(Pose2 pose, double length, double width) {
        this.pose = pose;
        double hl = length / 2.0;
        double hw = width / 2.0;
        this.halfLength = hl;
        this.halfWidth = hw;
        double[] f = {hl, -hl, -hl, hl};      // front-left, back-left, back-right, front-right
        double[] l = {hw, hw, -hw, -hw};
        for (int i = 0; i < 4; i++) {
            xs[i] = pose.fieldX(f[i], l[i]);
            ys[i] = pose.fieldY(f[i], l[i]);
        }
    }

    /** The footprint of the robot ({@link BioBuzzConstants.Robot} length and width) at a pose. */
    public static Footprint of(Pose2 pose) {
        return new Footprint(pose, BioBuzzConstants.Robot.LENGTH_IN, BioBuzzConstants.Robot.WIDTH_IN);
    }

    public static Footprint of(Pose2 pose, double length, double width) {
        return new Footprint(pose, length, width);
    }

    public Pose2 pose() {
        return pose;
    }

    /** Corner i (0 front-left, 1 back-left, 2 back-right, 3 front-right). */
    public Point2 corner(int i) {
        return new Point2(xs[i], ys[i]);
    }

    /** The axis-aligned bounding box. */
    public Rect bounds() {
        double minX = xs[0];
        double maxX = xs[0];
        double minY = ys[0];
        double maxY = ys[0];
        for (int i = 1; i < 4; i++) {
            minX = Math.min(minX, xs[i]);
            maxX = Math.max(maxX, xs[i]);
            minY = Math.min(minY, ys[i]);
            maxY = Math.max(maxY, ys[i]);
        }
        return new Rect(minX, maxX, minY, maxY);
    }

    /** True if the footprint and the rectangle share more than a boundary (separating-axis test). */
    public boolean overlaps(Rect r) {
        Rect b = bounds();
        if (b.maxX <= r.minX + EPS || b.minX >= r.maxX - EPS || b.maxY <= r.minY + EPS || b.minY >= r.maxY - EPS) {
            return false;
        }
        // the two edge normals of the robot rectangle are the remaining separating-axis candidates
        double c = Math.cos(pose.heading);
        double s = Math.sin(pose.heading);
        double[][] axes = {{c, s}, {-s, c}};
        double[] rx = {r.minX, r.maxX, r.maxX, r.minX};
        double[] ry = {r.minY, r.minY, r.maxY, r.maxY};
        for (double[] a : axes) {
            double aMin = Double.POSITIVE_INFINITY;
            double aMax = Double.NEGATIVE_INFINITY;
            double bMin = Double.POSITIVE_INFINITY;
            double bMax = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < 4; i++) {
                double pa = xs[i] * a[0] + ys[i] * a[1];
                aMin = Math.min(aMin, pa);
                aMax = Math.max(aMax, pa);
                double pb = rx[i] * a[0] + ry[i] * a[1];
                bMin = Math.min(bMin, pb);
                bMax = Math.max(bMax, pb);
            }
            if (aMax <= bMin + EPS || bMax <= aMin + EPS) {
                return false;
            }
        }
        return true;
    }

    /** How far the footprint sticks out of the rectangle: the largest overshoot over any side (0 if inside). */
    public double overshootOutside(Rect r) {
        Rect b = bounds();
        double over = 0.0;
        over = Math.max(over, r.minX - b.minX);
        over = Math.max(over, b.maxX - r.maxX);
        over = Math.max(over, r.minY - b.minY);
        over = Math.max(over, b.maxY - r.maxY);
        return over;
    }

    /** True if the footprint lies inside the rectangle grown by {@code tolerance} on every side. */
    public boolean insideRect(Rect r, double tolerance) {
        return overshootOutside(r) <= tolerance + EPS;
    }

    /**
     * The distance from the robot's rotation center to its farthest corner, in inches (half the diagonal of the
     * {@code LENGTH x WIDTH} rectangle: 12.59 in for the 17.80 in square). Every point of the robot is at most this far
     * from the center, so turning the robot by an angle t moves no point of it by more than {@code halfDiagonal * t}.
     * The route validator relies on this to prove that its samples cannot miss a collision (see {@code Sweep}).
     */
    public static double robotHalfDiagonal() {
        return Math.hypot(BioBuzzConstants.Robot.LENGTH_IN / 2.0, BioBuzzConstants.Robot.WIDTH_IN / 2.0);
    }

    /**
     * The chassis at a pose: the robot footprint WITHOUT its front intake strip (the front {@code intakeReach} inches,
     * where the flat intake wheels stick out). The rectangle is {@code LENGTH - intakeReach} long and its front edge is
     * {@code intakeReach} behind the footprint's front edge.
     */
    public static Footprint chassis(Pose2 pose, double intakeReach) {
        double length = BioBuzzConstants.Robot.LENGTH_IN - intakeReach;
        // the chassis is centered half the reach BEHIND the robot's center
        Pose2 center = new Pose2(pose.fieldX(-intakeReach / 2.0, 0.0), pose.fieldY(-intakeReach / 2.0, 0.0), pose.heading);
        return new Footprint(center, length, BioBuzzConstants.Robot.WIDTH_IN);
    }

    /**
     * The front intake strip at a pose: the front {@code intakeReach} inches of the footprint, full width. Together
     * with {@link #chassis} it covers the whole {@link #of(Pose2) footprint} exactly (the two only share an edge).
     */
    public static Footprint intakeStrip(Pose2 pose, double intakeReach) {
        double forward = (BioBuzzConstants.Robot.LENGTH_IN - intakeReach) / 2.0;
        Pose2 center = new Pose2(pose.fieldX(forward, 0.0), pose.fieldY(forward, 0.0), pose.heading);
        return new Footprint(center, intakeReach, BioBuzzConstants.Robot.WIDTH_IN);
    }

    /**
     * How far the footprint is outside the rectangle, with a sign: POSITIVE = the worst corner is that many inches
     * outside (past a wall), NEGATIVE = every corner is inside and the nearest one is that many inches from the
     * nearest wall, 0 = touching. Unlike {@link #overshootOutside} it does not stop at 0, which is what lets the
     * sweep check measure "how far from trouble" while the robot is still clear.
     */
    public double signedOvershoot(Rect r) {
        Rect b = bounds();
        return Math.max(Math.max(r.minX - b.minX, b.maxX - r.maxX), Math.max(r.minY - b.minY, b.maxY - r.maxY));
    }

    /**
     * The signed gap between the footprint and a rectangle: POSITIVE = they are that many inches apart (the shortest
     * distance between the two shapes), 0 = they just touch, NEGATIVE = they overlap and this is how deep (the
     * smallest distance the footprint would have to slide to get free). It is a true distance, so if every point of the
     * robot moves by at most m inches the gap changes by at most m: the property the sweep guarantee is built on.
     */
    public double signedGap(Rect r) {
        // Separating-axis test. For convex shapes, the best (largest) separation over the four edge directions is
        // negative exactly when they overlap, and then it is minus the depth of the overlap.
        double c = Math.cos(pose.heading);
        double s = Math.sin(pose.heading);
        double[][] axes = {{1.0, 0.0}, {0.0, 1.0}, {c, s}, {-s, c}};
        double[] rx = {r.minX, r.maxX, r.maxX, r.minX};
        double[] ry = {r.minY, r.minY, r.maxY, r.maxY};
        double best = Double.NEGATIVE_INFINITY;
        for (double[] a : axes) {
            double aMin = Double.POSITIVE_INFINITY;
            double aMax = Double.NEGATIVE_INFINITY;
            double bMin = Double.POSITIVE_INFINITY;
            double bMax = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < 4; i++) {
                double pa = xs[i] * a[0] + ys[i] * a[1];
                aMin = Math.min(aMin, pa);
                aMax = Math.max(aMax, pa);
                double pb = rx[i] * a[0] + ry[i] * a[1];
                bMin = Math.min(bMin, pb);
                bMax = Math.max(bMax, pb);
            }
            best = Math.max(best, Math.max(aMin - bMax, bMin - aMax));
        }
        if (best < 0.0) {
            return best;
        }
        // Apart: the true distance is between a corner of one shape and the other shape.
        double nearest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 4; i++) {
            double dx = Math.max(Math.max(r.minX - xs[i], 0.0), xs[i] - r.maxX);
            double dy = Math.max(Math.max(r.minY - ys[i], 0.0), ys[i] - r.maxY);
            nearest = Math.min(nearest, Math.hypot(dx, dy));
            double forward = pose.forwardOf(rx[i], ry[i]);
            double left = pose.leftOf(rx[i], ry[i]);
            double fx = Math.max(Math.abs(forward) - halfLength, 0.0);
            double fy = Math.max(Math.abs(left) - halfWidth, 0.0);
            nearest = Math.min(nearest, Math.hypot(fx, fy));
        }
        return nearest;
    }

    /**
     * The bounding box of the part of the footprint that lies inside the rectangle (convex polygon
     * clipping), or null if they do not overlap.
     */
    public Rect clippedBounds(Rect r) {
        List<double[]> poly = new ArrayList<double[]>();
        for (int i = 0; i < 4; i++) {
            poly.add(new double[] {xs[i], ys[i]});
        }
        poly = clip(poly, 0, r.minX, true);
        poly = clip(poly, 0, r.maxX, false);
        poly = clip(poly, 1, r.minY, true);
        poly = clip(poly, 1, r.maxY, false);
        if (poly.size() < 3) {
            return null;
        }
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        for (double[] p : poly) {
            minX = Math.min(minX, p[0]);
            maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]);
            maxY = Math.max(maxY, p[1]);
        }
        if (maxX - minX < EPS || maxY - minY < EPS) {
            return null;
        }
        return new Rect(minX, maxX, minY, maxY);
    }

    /** Sutherland-Hodgman step: keep the part of the polygon on one side of an axis-aligned line. */
    private static List<double[]> clip(List<double[]> in, int axis, double limit, boolean keepGreater) {
        List<double[]> out = new ArrayList<double[]>();
        int n = in.size();
        for (int i = 0; i < n; i++) {
            double[] a = in.get(i);
            double[] b = in.get((i + 1) % n);
            boolean aIn = keepGreater ? a[axis] >= limit : a[axis] <= limit;
            boolean bIn = keepGreater ? b[axis] >= limit : b[axis] <= limit;
            if (aIn) {
                out.add(a);
            }
            if (aIn != bIn) {
                double t = (limit - a[axis]) / (b[axis] - a[axis]);
                out.add(new double[] {a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t});
            }
        }
        return out;
    }
}
