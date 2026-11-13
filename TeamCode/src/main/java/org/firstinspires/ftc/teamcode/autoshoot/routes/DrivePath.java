package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;

/**
 * The geometry of one DRIVE leg: a straight line or a Bezier curve (the via points are its control
 * points) plus the heading the robot has at every point of it, following the leg's
 * {@link HeadingMode}. The validator samples it, the timing model measures it, the renderer draws it.
 *
 * <p>The path starts at {@code from}, the pose where the robot is when the leg begins (the end pose
 * of the previous step). That pose's heading matters: LINEAR turns away from it, and the difference
 * between it and the heading the leg wants at its first point is a turn in place before the leg.
 */
public final class DrivePath {
    private static final double TINY = 1e-9;
    /** A numeric guard, not a tuning value: the heading cut loop never halves one interval more often than this. */
    private static final int MAX_CUT_DEPTH = 24;

    private final Pose2 from;
    private final List<Point2> control;
    private final HeadingMode mode;
    private final Pose2 target;
    private final Point2 facePoint;
    private final boolean faceWithBack;

    // How dense the table of the curve is: this many nodes per inch of its control polygon, but never fewer than the smallest or more than the
    // largest table. This is numeric resolution (a node every quarter inch measures a curve to well under a thousandth), not a tuning knob.
    private static final double TABLE_NODES_PER_IN = 4.0;
    private static final int TABLE_MIN_NODES = 64;
    private static final int TABLE_MAX_NODES = 4000;

    // dense table of the curve: parameter t and cumulative arc length s at each node
    private final double[] tTable;
    private final double[] sTable;
    private final double length;

    private DrivePath(Pose2 from, List<Point2> control, HeadingMode mode, Pose2 target, Point2 facePoint,
                      boolean faceWithBack) {
        this.from = from;
        this.control = Collections.unmodifiableList(new ArrayList<Point2>(control));
        this.mode = mode;
        this.target = target;
        this.facePoint = facePoint;
        this.faceWithBack = faceWithBack;

        double polyLength = 0.0;
        for (int i = 1; i < control.size(); i++) {
            polyLength += control.get(i - 1).distanceTo(control.get(i));
        }
        int n = (int) Math.max(TABLE_MIN_NODES, Math.min(TABLE_MAX_NODES, Math.ceil(polyLength * TABLE_NODES_PER_IN)));
        tTable = new double[n + 1];
        sTable = new double[n + 1];
        Point2 prev = bezier(0.0);
        for (int i = 1; i <= n; i++) {
            double t = (double) i / n;
            Point2 p = bezier(t);
            tTable[i] = t;
            sTable[i] = sTable[i - 1] + prev.distanceTo(p);
            prev = p;
        }
        length = sTable[n];
    }

    /** The path of a DRIVE step, beginning at {@code from}. */
    public static DrivePath of(Pose2 from, Step.Drive step) {
        List<Point2> pts = new ArrayList<Point2>();
        pts.add(new Point2(from.x, from.y));
        pts.addAll(step.via());
        pts.add(new Point2(step.target().x, step.target().y));
        return new DrivePath(from, pts, step.mode(), step.target(), step.facePoint(), step.faceWithBack());
    }

    /**
     * A straight move from one pose to another with the heading turning evenly. This is the implicit
     * move the solver or the hold command makes to reach an INTAKE, SHOOT or PARK pose.
     */
    public static DrivePath straight(Pose2 from, Pose2 to) {
        List<Point2> pts = new ArrayList<Point2>();
        pts.add(new Point2(from.x, from.y));
        pts.add(new Point2(to.x, to.y));
        return new DrivePath(from, pts, HeadingMode.LINEAR, to, null, false);
    }

    /** Arc length of the whole path in inches. */
    public double length() {
        return length;
    }

    public HeadingMode mode() {
        return mode;
    }

    /** The control polygon: the start point, the via points, the end point. */
    public List<Point2> controlPoints() {
        return control;
    }

    /** 1 for a straight line, 2 for a quadratic Bezier (one via point), 3 for a cubic, ... */
    public int degree() {
        return control.size() - 1;
    }

    // ---- geometry ----------------------------------------------------------------------------------

    private Point2 bezier(double t) {
        int n = control.size();
        double[] xs = new double[n];
        double[] ys = new double[n];
        for (int i = 0; i < n; i++) {
            xs[i] = control.get(i).x;
            ys[i] = control.get(i).y;
        }
        for (int level = n - 1; level > 0; level--) {
            for (int i = 0; i < level; i++) {
                xs[i] += (xs[i + 1] - xs[i]) * t;
                ys[i] += (ys[i + 1] - ys[i]) * t;
            }
        }
        return new Point2(xs[0], ys[0]);
    }

    /** The derivative of the Bezier at t (not normalized). */
    private Point2 derivative(double t) {
        int n = control.size() - 1;      // degree
        if (n < 1) {
            return new Point2(0, 0);
        }
        // derivative control points: n * (P[i+1] - P[i]), a Bezier of degree n-1
        double[] xs = new double[n];
        double[] ys = new double[n];
        for (int i = 0; i < n; i++) {
            xs[i] = n * (control.get(i + 1).x - control.get(i).x);
            ys[i] = n * (control.get(i + 1).y - control.get(i).y);
        }
        for (int level = n - 1; level > 0; level--) {
            for (int i = 0; i < level; i++) {
                xs[i] += (xs[i + 1] - xs[i]) * t;
                ys[i] += (ys[i + 1] - ys[i]) * t;
            }
        }
        return new Point2(xs[0], ys[0]);
    }

    /** The curve parameter t at arc length s (clamped to the path). */
    private double tAt(double s) {
        if (length < TINY) {
            return 0.0;
        }
        double c = Math.max(0.0, Math.min(length, s));
        int lo = 0;
        int hi = sTable.length - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (sTable[mid] <= c) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        double span = sTable[hi] - sTable[lo];
        double f = span < TINY ? 0.0 : (c - sTable[lo]) / span;
        return tTable[lo] + f * (tTable[hi] - tTable[lo]);
    }

    /** The point at arc length s (clamped to [0, length]). */
    public Point2 pointAt(double s) {
        return bezier(tAt(s));
    }

    /** Direction of travel at arc length s, in radians; the previous heading if the path has no length. */
    private double travelDirection(double s) {
        if (length < TINY) {
            return from.heading;
        }
        double t = tAt(s);
        Point2 d = derivative(t);
        if (Math.hypot(d.x, d.y) < 1e-6) {
            // the control points coincide at this end: use a point slightly further along instead
            double back = Math.max(0.0, s - 0.5);
            double fwd = Math.min(length, s + 0.5);
            Point2 a = bezier(tAt(back));
            Point2 b = bezier(tAt(fwd));
            d = new Point2(b.x - a.x, b.y - a.y);
            if (Math.hypot(d.x, d.y) < 1e-9) {
                return from.heading;
            }
        }
        return Math.atan2(d.y, d.x);
    }

    /** The robot heading, in radians, at arc length s. */
    public double headingAt(double s) {
        double frac = length < TINY ? 1.0 : Math.max(0.0, Math.min(1.0, s / length));
        switch (mode) {
            case TANGENT:
                return Angles.normalize(travelDirection(s));
            case REVERSE_TANGENT:
                return Angles.normalize(travelDirection(s) + Math.PI);
            case LINEAR:
                return Angles.normalize(from.heading + Angles.error(from.heading, target.heading) * frac);
            case CONSTANT:
                return target.heading;
            case FACE_POINT: {
                Point2 p = pointAt(s);
                if (p.distanceTo(facePoint) < TINY) {
                    return from.heading;
                }
                double toPoint = Math.atan2(facePoint.y - p.y, facePoint.x - p.x);
                return Angles.normalize(faceWithBack ? toPoint + Math.PI : toPoint);
            }
            default:
                throw new IllegalStateException("unhandled heading mode " + mode);
        }
    }

    public Pose2 poseAt(double s) {
        Point2 p = pointAt(s);
        return new Pose2(p.x, p.y, headingAt(s));
    }

    /** The pose the leg starts at: the start point with the heading the leg wants there. */
    public Pose2 start() {
        return poseAt(0.0);
    }

    /** The pose the robot ends the leg in (the effective end heading follows from the heading mode). */
    public Pose2 end() {
        return poseAt(length);
    }

    // ---- turning -----------------------------------------------------------------------------------

    /** How far the robot turns in place before the leg starts moving: previous heading -> heading at s = 0. */
    public double preTurnRadians() {
        return Math.abs(Angles.error(from.heading, headingAt(0.0)));
    }

    /** Poses at the start point while the robot turns in place, every {@code stepRadians}; empty if no turn is needed. */
    public List<Pose2> turnInPlaceSamples(double stepRadians) {
        List<Pose2> out = new ArrayList<Pose2>();
        double target0 = headingAt(0.0);
        double total = Angles.error(from.heading, target0);
        if (Math.abs(total) < TINY) {
            return out;
        }
        int n = (int) Math.ceil(Math.abs(total) / stepRadians);
        Point2 p0 = pointAt(0.0);
        for (int i = 1; i <= n; i++) {
            out.add(new Pose2(p0.x, p0.y, from.heading + total * i / n));
        }
        return out;
    }

    /**
     * The signed turn in place before the leg starts moving, in radians (positive = counter-clockwise): from the
     * heading the robot has to {@link #headingAt headingAt(0)}. Zero if the robot already faces the right way.
     */
    public double preTurnSignedRadians() {
        return Angles.error(from.heading, headingAt(0.0));
    }

    /** The pose at the start point after the robot has turned {@code turned} radians of the pre-turn (0 = not yet). */
    public Pose2 turnPoseAt(double turned) {
        Point2 p0 = pointAt(0.0);
        double total = preTurnSignedRadians();
        return new Pose2(p0.x, p0.y, from.heading + Math.copySign(turned, total));
    }

    /** All the turning of the leg: the turn in place at the start plus every heading change along the way. */
    public double totalTurnRadians() {
        double total = preTurnRadians();
        double prev = headingAt(0.0);
        List<Pose2> samples = sample(1.0);
        for (Pose2 p : samples) {
            total += Math.abs(Angles.error(prev, p.heading));
            prev = p.heading;
        }
        return total;
    }

    // ---- sampling ----------------------------------------------------------------------------------

    /**
     * Poses along the path every {@code stepIn} inches of arc length, always including the start and the
     * end. A path of length 0 yields a single pose.
     */
    public List<Pose2> sample(double stepIn) {
        if (!(stepIn > 0.0)) {
            throw new IllegalArgumentException("sample step must be > 0");
        }
        List<Pose2> out = new ArrayList<Pose2>();
        if (length < TINY) {
            out.add(poseAt(0.0));
            return out;
        }
        int n = (int) Math.floor(length / stepIn + 1e-9);
        for (int i = 0; i <= n; i++) {
            out.add(poseAt(Math.min(length, i * stepIn)));
        }
        if (length - n * stepIn > 1e-6) {
            out.add(poseAt(length));
        }
        return out;
    }

    /** Arc lengths belonging to {@link #sample}, same order. */
    public List<Double> sampleArcLengths(double stepIn) {
        List<Double> out = new ArrayList<Double>();
        if (length < TINY) {
            out.add(0.0);
            return out;
        }
        int n = (int) Math.floor(length / stepIn + 1e-9);
        for (int i = 0; i <= n; i++) {
            out.add(Math.min(length, i * stepIn));
        }
        if (length - n * stepIn > 1e-6) {
            out.add(length);
        }
        return out;
    }

    /**
     * As {@link #sampleArcLengths(double)}, and ALSO cut the path wherever the heading would change by more than
     * {@code maxHeadingStepRad} between two neighbouring samples. A leg that turns a lot over a short distance would
     * otherwise be checked only at its two ends while the robot's corners swing through a big arc in between. After
     * this call the neighbouring samples are at most {@code stepIn} apart along the path AND at most
     * {@code maxHeadingStepRad} apart in heading, and the heading does not go one way and come back inside one gap:
     * the two facts the sweep guarantee in {@code Sweep} needs. The start and the end are always included.
     */
    public List<Double> sampleArcLengths(double stepIn, double maxHeadingStepRad) {
        if (!(maxHeadingStepRad > 0.0)) {
            throw new IllegalArgumentException("heading step must be > 0");
        }
        List<Double> base = sampleArcLengths(stepIn);
        List<Double> out = new ArrayList<Double>();
        out.add(base.get(0));
        for (int i = 1; i < base.size(); i++) {
            cut(base.get(i - 1), base.get(i), maxHeadingStepRad, out, 0);
            out.add(base.get(i));
        }
        return out;
    }

    /** Poses at the arc lengths of {@link #sampleArcLengths(double, double)}. */
    public List<Pose2> sample(double stepIn, double maxHeadingStepRad) {
        List<Pose2> out = new ArrayList<Pose2>();
        for (double s : sampleArcLengths(stepIn, maxHeadingStepRad)) {
            out.add(poseAt(s));
        }
        return out;
    }

    /** Adds the arc lengths strictly between a and b that make every gap turn by at most {@code maxStep}. */
    private void cut(double a, double b, double maxStep, List<Double> out, int depth) {
        if (depth >= MAX_CUT_DEPTH || b - a < TINY) {
            return;
        }
        double m = (a + b) / 2.0;
        double first = Angles.error(headingAt(a), headingAt(m));
        double second = Angles.error(headingAt(m), headingAt(b));
        boolean tooBig = Math.abs(first + second) > maxStep || Math.abs(first) > maxStep || Math.abs(second) > maxStep;
        // the heading went one way and came back inside the gap: the two ends alone would hide the swing
        boolean wiggle = first * second < 0.0 && Math.abs(first) + Math.abs(second) > maxStep;
        if (!tooBig && !wiggle) {
            return;
        }
        cut(a, m, maxStep, out, depth + 1);
        out.add(m);
        cut(m, b, maxStep, out, depth + 1);
    }
}
