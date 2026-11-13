package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The robot's floor footprint at a pose: a rectangle {@code LENGTH x WIDTH} (front to back by side to
 * side) rotated to the pose's heading about the robot's rotation center. The corners run
 * counter-clockwise starting at the front-left one.
 *
 * <p>Three shapes come out of this class, all rectangles that turn with the robot:
 * <ul>
 *   <li>{@link #of(Pose2)}: the whole PERIMETER (every wheel included). Walls, the HIVE keep-out and the rest of the
 *       field are judged with this one.</li>
 *   <li>{@link #chassis(Pose2)}: the BODY, the perimeter without the front {@code Robot.INTAKE_REACH_IN} where only the
 *       two flat intake wheels stick out. A FLOWER plate must never touch it, not even while the wheels press in.</li>
 *   <li>{@link #wheelDisc(Pose2, IntakeWheel)}: one of the two flat intake WHEELS, a small square around the wheel's
 *       disc. It is the only part of the robot that may enter a FLOWER plate, and only to press the POLLEN.</li>
 * </ul>
 *
 * <p>The rectangle does not have to be centered on the pose point: it is described by how far it reaches forward,
 * backward, left and right of that point, so the body and the wheel discs (which sit off to the side and at the very
 * front) use the same maths as the perimeter. Where the perimeter lies around the pose point is written in ONE place,
 * {@link #frontExtentIn()} and its three siblings; everything else in the code asks them (nobody writes
 * {@code LENGTH_IN / 2} anywhere else), so the day the real center of rotation is measured those four are all that change.
 *
 * <p>All overlap tests treat touching as NOT overlapping (a robot may stand against a wall or a
 * keep-out edge); a penetration of more than a nanoinch counts.
 */
public final class Footprint {
    private static final double EPS = 1e-9;

    private final Pose2 pose;
    /** The rectangle in the robot frame: how far it reaches forward / backward / left / right of the pose point (inches). */
    private final double forwardMax;
    private final double forwardMin;
    private final double leftMax;
    private final double leftMin;
    private final double[] xs = new double[4];
    private final double[] ys = new double[4];

    private Footprint(Pose2 pose, double forwardMin, double forwardMax, double leftMin, double leftMax) {
        this.pose = pose;
        this.forwardMin = forwardMin;
        this.forwardMax = forwardMax;
        this.leftMin = leftMin;
        this.leftMax = leftMax;
        double[] f = {forwardMax, forwardMin, forwardMin, forwardMax};      // front-left, back-left, back-right, front-right
        double[] l = {leftMax, leftMax, leftMin, leftMin};
        for (int i = 0; i < 4; i++) {
            xs[i] = pose.fieldX(f[i], l[i]);
            ys[i] = pose.fieldY(f[i], l[i]);
        }
    }

    // ---- where the perimeter lies around the pose point (the one place that knows) ------------------------

    /**
     * How far the perimeter reaches IN FRONT of the pose point, in inches: the tips of the flat intake wheels. Today the
     * pose point is taken to be the middle of the {@code LENGTH_IN x WIDTH_IN} rectangle, so this is half the length.
     */
    public static double frontExtentIn() {
        return BioBuzzConstants.Robot.LENGTH_IN / 2.0;
    }

    /** How far the perimeter reaches BEHIND the pose point, in inches (the back lip, where the ball leaves). */
    public static double rearExtentIn() {
        return BioBuzzConstants.Robot.LENGTH_IN / 2.0;
    }

    /** How far the perimeter reaches to the robot's LEFT of the pose point, in inches. */
    public static double leftExtentIn() {
        return BioBuzzConstants.Robot.WIDTH_IN / 2.0;
    }

    /** How far the perimeter reaches to the robot's RIGHT of the pose point, in inches. */
    public static double rightExtentIn() {
        return BioBuzzConstants.Robot.WIDTH_IN / 2.0;
    }

    // ---- the three shapes -------------------------------------------------------------------------------

    /** The whole footprint of the robot (its perimeter, wheels included) at a pose. */
    public static Footprint of(Pose2 pose) {
        return new Footprint(pose, -rearExtentIn(), frontExtentIn(), -rightExtentIn(), leftExtentIn());
    }

    /** A rectangle {@code length x width} centered on the pose point (used by tests and tools). */
    public static Footprint of(Pose2 pose, double length, double width) {
        return new Footprint(pose, -length / 2.0, length / 2.0, -width / 2.0, width / 2.0);
    }

    /**
     * The chassis at a pose: the perimeter WITHOUT its front {@code intakeReach} inches, the stretch where only the two
     * flat intake wheels stick out in front of the frame. Its front edge is {@code intakeReach} behind the front of the
     * {@link #of(Pose2) footprint}; the other three edges are the footprint's.
     */
    public static Footprint chassis(Pose2 pose, double intakeReach) {
        return new Footprint(pose, -rearExtentIn(), frontExtentIn() - intakeReach, -rightExtentIn(), leftExtentIn());
    }

    /** The chassis with the real wheel reach, {@code Robot.INTAKE_REACH_IN}. */
    public static Footprint chassis(Pose2 pose) {
        return chassis(pose, BioBuzzConstants.Robot.INTAKE_REACH_IN);
    }

    /**
     * One flat intake wheel at a pose: the smallest square that holds the wheel's disc (side
     * {@code Robot.INTAKE_WHEEL_DIAMETER_IN}), centered {@code Robot.INTAKE_WHEEL_OFFSET_IN} to the side of the robot's
     * center line, with its front edge on the front of the perimeter. A square is a little bigger than the round disc, so
     * a robot that passes the check with the square is certainly clear with the real wheel.
     */
    public static Footprint wheelDisc(Pose2 pose, IntakeWheel wheel) {
        double d = BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN;
        double side = wheel.sign() * BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN;
        return new Footprint(pose, frontExtentIn() - d, frontExtentIn(), side - d / 2.0, side + d / 2.0);
    }

    /**
     * Where the middle of a flat intake wheel is, in field inches: {@code Robot.INTAKE_WHEEL_OFFSET_IN} to the side of the
     * center line and half a wheel diameter behind the front of the perimeter. This is the point that has to be on the
     * POLLEN when a FLOWER is pressed.
     */
    public static Point2 wheelCenter(Pose2 pose, IntakeWheel wheel) {
        double forward = frontExtentIn() - BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0;
        double left = wheel.sign() * BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN;
        return new Point2(pose.fieldX(forward, left), pose.fieldY(forward, left));
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
     * {@code LENGTH x WIDTH} rectangle: 12.59 in for the 17.80 in square centered on the pose point). Every point of the
     * robot, the chassis and the wheel discs included, is at most this far from the center, so turning the robot by an
     * angle t moves no point of it by more than {@code halfDiagonal * t}. The route validator relies on this to prove that
     * its samples cannot miss a collision (see {@code Sweep}).
     */
    public static double robotHalfDiagonal() {
        return Math.hypot(Math.max(frontExtentIn(), rearExtentIn()), Math.max(leftExtentIn(), rightExtentIn()));
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
            double fx = Math.max(Math.max(forwardMin - forward, forward - forwardMax), 0.0);
            double fy = Math.max(Math.max(leftMin - left, left - leftMax), 0.0);
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
