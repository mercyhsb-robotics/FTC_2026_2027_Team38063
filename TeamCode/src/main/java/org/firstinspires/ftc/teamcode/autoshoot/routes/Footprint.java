package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;

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
    private final double[] xs = new double[4];
    private final double[] ys = new double[4];

    private Footprint(Pose2 pose, double length, double width) {
        this.pose = pose;
        double hl = length / 2.0;
        double hw = width / 2.0;
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
