package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Checks that a whole driven route keeps the robot's real rectangle off the field walls, the HIVE frame and the
 * FLOWER plates, at EVERY point of the route, including while the robot is turning.
 *
 * <p>Why it exists. {@link PathPlanner} plans a route for the robot's CENTER, a single dot. It keeps the dot away from
 * the walls by about the robot's smallest half-size, which is enough while the robot faces along an axis. But a 17.8 in
 * square-ish robot that turns takes more room: standing flush against a wall it needs its center of rotation 8.03 to
 * 9.77 in from the wall (the rear, the sides or the front edge touching, see {@link RobotShape}), and turned 45 degrees
 * it needs about 13.2 in, because a corner leads. The robot does turn while it drives (see {@link HeadingProfile}), so a
 * route that is fine for a dot can push a corner of the robot THROUGH a wall. Standing poses were already tested with
 * the real rotated rectangle ({@link Footprint}); this class does the same for everything in between.
 *
 * <p>How it works, in plain words:
 * <ol>
 *   <li>It asks {@link HeadingProfile} for the robot's pose (position and heading) at a point of the route and builds
 *       the real rotated rectangle there. That is one "sample".</li>
 *   <li>For each thing to stay away from it computes the <b>gap</b>: how many inches of clear space are between the
 *       rectangle and that thing. Positive means clear, zero means touching, negative means overlapping (the number is
 *       how deep). The four walls and each obstacle are separate things.</li>
 *   <li>The route is split into pieces (at least every {@code stepIn} inches and every {@code stepDeg} degrees of turn).
 *       A piece is declared safe only when it can be PROVEN safe from its two end samples: while the robot moves between
 *       two samples, its farthest corner moves at most "the distance driven plus the corner's radius times the angle
 *       turned" (call this the <b>corner travel</b>), so no part of the robot gets closer to anything than the smaller
 *       end gap minus half the corner travel. If that is not enough room, the piece is cut in half and each half is
 *       tried again, down to {@code minStepIn}. So the robot is looked at often exactly where it is close to something,
 *       and a thin obstacle cannot slip between two samples.</li>
 *   <li>A sample fails when a gap is smaller than what is required, see below.</li>
 * </ol>
 *
 * <p><b>What gap is required.</b> For each wall the robot must keep {@code wallClearanceIn} (the same sliver of empty
 * space standing poses keep); for each obstacle it must keep {@code obstacleMarginIn}. There is one exception, which
 * makes the check usable: the route is allowed to START inside that margin, because a robot that begins the match
 * touching a wall is already closer than the clearance. For every wall or obstacle the required gap is therefore the
 * SMALLER of the clearance and the gap the robot has at the start of the route: <i>a route may never squeeze the robot
 * harder against something than it already is at the start</i>. A robot that starts clear must stay clear; a robot that
 * starts touching the audience wall may keep touching it but may never push through it. The end of the route is a
 * standing pose that the planner already checked, so it needs no exception. Finally the checker forgives
 * {@code toleranceIn} (a few hundredths of an inch) for rounding.
 *
 * <p>Units and frame: Pedro field frame, inches and radians. Pure Java.
 */
public final class RouteSweep {

    /** How finely a route is looked at; see the constants in {@link BioBuzzConstants.AutoShoot} (names start PATH_SWEEP_). */
    public static final class Settings {
        /** The robot is looked at at least every this many inches of driving. */
        public double stepIn;
        /** ... and at least every this many degrees of turning. */
        public double stepDeg;
        /** A gap this much (inches) below the required one is forgiven, for rounding. */
        public double toleranceIn;
        /** Pieces are never cut shorter than this many inches. */
        public double minStepIn;

        /** The settings the robot runs with. */
        public static Settings fromConstants() {
            Settings s = new Settings();
            s.stepIn = BioBuzzConstants.AutoShoot.PATH_SWEEP_STEP_IN;
            s.stepDeg = BioBuzzConstants.AutoShoot.PATH_SWEEP_STEP_DEG;
            s.toleranceIn = BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN;
            s.minStepIn = BioBuzzConstants.AutoShoot.PATH_SWEEP_MIN_STEP_IN;
            return s;
        }
    }

    /** The first place a route fails the check. */
    public static final class Violation {
        /** Inches driven from the start of the route to the failing sample. */
        public final double distanceIn;
        /** The robot's pose at that sample. */
        public final Pose2 pose;
        /** What the robot got too close to, for example "wall x = 0.0" or "obstacle 0 x[40.00,60.00] y[40.00,60.00]". */
        public final String what;
        /** The gap at that sample, inches (negative = the robot overlaps it by that much). */
        public final double gapIn;
        /** The gap the route had to keep, inches. */
        public final double requiredIn;

        Violation(double distanceIn, Pose2 pose, String what, double gapIn, double requiredIn) {
            this.distanceIn = distanceIn;
            this.pose = pose;
            this.what = what;
            this.gapIn = gapIn;
            this.requiredIn = requiredIn;
        }

        @Override
        public String toString() {
            return String.format(Locale.ROOT, "%.2f in along the route at %s: gap to %s is %.2f in, needs %.2f in",
                    distanceIn, pose, what, gapIn, requiredIn);
        }
    }

    private final Rect field;
    private final double wallClearanceIn;
    private final List<Rect> obstacles;
    private final double obstacleMarginIn;
    private final RobotShape robot;
    private final Settings settings;
    // The farthest any point of the robot is from its rotation center (inches): the radius of the smallest circle that
    // holds the robot whatever its heading. It sets how fast a corner can move while the robot turns.
    private final double reachIn;
    // One entry per thing to stay away from: the four walls first, then the obstacles.
    private final int count;
    private final String[] names;
    private final double[] clearance;

    /**
     * @param field            the whole field (the walls are its four sides)
     * @param wallClearanceIn  empty space kept between the robot and a wall, inches
     * @param obstacles        the REAL keep-out boxes (the HIVE frame and the FLOWER plates), not yet grown by any margin
     * @param obstacleMarginIn empty space kept between the robot and an obstacle, inches
     * @param robot            where the robot's edges lie around its center of rotation (see {@link RobotShape})
     * @param settings         how finely to look; see {@link Settings}
     */
    public RouteSweep(Rect field, double wallClearanceIn, List<Rect> obstacles, double obstacleMarginIn,
                      RobotShape robot, Settings settings) {
        this.field = field;
        this.wallClearanceIn = wallClearanceIn;
        this.obstacles = new ArrayList<Rect>(obstacles);
        this.obstacleMarginIn = obstacleMarginIn;
        this.robot = robot;
        this.settings = settings;

        // The farthest corner of the upright rectangle from the center of rotation. It is measured on a real Footprint
        // (not copied from the shape) so the sweep and the checks can never disagree about how far the robot reaches.
        Footprint upright = Footprint.of(new Pose2(0.0, 0.0, 0.0), robot);
        double reach = 0.0;
        for (int i = 0; i < 4; i++) {
            reach = Math.max(reach, Math.hypot(upright.corner(i).x, upright.corner(i).y));
        }
        this.reachIn = reach;

        count = 4 + this.obstacles.size();
        names = new String[count];
        clearance = new double[count];
        names[0] = String.format(Locale.ROOT, "wall x = %.1f", field.minX);
        names[1] = String.format(Locale.ROOT, "wall x = %.1f", field.maxX);
        names[2] = String.format(Locale.ROOT, "wall y = %.1f", field.minY);
        names[3] = String.format(Locale.ROOT, "wall y = %.1f", field.maxY);
        for (int i = 0; i < 4; i++) {
            clearance[i] = wallClearanceIn;
        }
        for (int k = 0; k < this.obstacles.size(); k++) {
            names[4 + k] = "obstacle " + k + " " + this.obstacles.get(k);
            clearance[4 + k] = obstacleMarginIn;
        }
    }

    /** The radius, in inches, of the smallest circle around the robot's rotation center that holds the whole robot. */
    public double reachIn() {
        return reachIn;
    }

    /**
     * True if, with its rotation center at (x, y), the robot may face ANY direction without coming closer than the
     * wall clearance to a wall or the margin to an obstacle. Equivalent to asking whether the circle of radius
     * {@link #reachIn} around the point is clear (a circle is what the robot sweeps out when it spins in place). The
     * planner uses it to find the stretches of a route where the robot is free to turn.
     */
    public boolean anyHeadingIsSafeAt(double x, double y) {
        // The 1e-9 forgives rounding for a point exactly on the limit.
        if (x - reachIn < field.minX + wallClearanceIn - 1e-9 || x + reachIn > field.maxX - wallClearanceIn + 1e-9
                || y - reachIn < field.minY + wallClearanceIn - 1e-9 || y + reachIn > field.maxY - wallClearanceIn + 1e-9) {
            return false;
        }
        for (Rect o : obstacles) {
            // A box grown by the robot's reach plus the margin is where the center may not be. (It grows corners
            // square rather than round, which is a little stricter than needed, never looser.)
            if (o.inflate(reachIn + obstacleMarginIn - 1e-9).containsStrictly(x, y)) {
                return false;
            }
        }
        return true;
    }

    /** The robot's rectangle at a pose, built exactly the way every other check builds it. */
    private Footprint footprintAt(Pose2 pose) {
        return Footprint.of(pose, robot);
    }

    /**
     * Checks a whole route.
     *
     * @return null if the robot's rectangle stays clear everywhere along the route (see the class note for what clear
     *         means); otherwise the first place it does not
     */
    public Violation check(HeadingProfile profile) {
        if (profile.isEmpty()) {
            return null;                    // nothing to drive: the robot does not move
        }
        // The gaps at the start decide what is required: never squeeze harder than at the start (class note).
        double[] startGaps = gapsAt(profile, 0.0);
        double[] need = new double[count];
        for (int c = 0; c < count; c++) {
            need[c] = Math.min(clearance[c], startGaps[c]) - settings.toleranceIn;
        }
        double stepRad = Math.toRadians(settings.stepDeg);
        double[] previous = startGaps;
        for (int leg = 0; leg < profile.vertexCount() - 1; leg++) {
            double sA = profile.distanceAtVertex(leg);
            double sB = profile.distanceAtVertex(leg + 1);
            double length = sB - sA;
            double turn = Math.abs(profile.headingAtVertex(leg + 1) - profile.headingAtVertex(leg));
            // Turn rate along this leg (radians per inch): constant, because the profile blends linearly.
            double rate = turn / length;
            // Cut the leg into equal pieces no longer than the step and no more turning than the heading step. The
            // 1e-9 forgives rounding so a leg of exactly one step is not cut in two.
            int pieces = (int) Math.max(1.0, Math.max(Math.ceil(length / settings.stepIn - 1e-9),
                    Math.ceil(turn / stepRad - 1e-9)));
            double sPrev = sA;
            for (int k = 1; k <= pieces; k++) {
                double s = sA + length * k / pieces;
                double[] here = gapsAt(profile, s);
                Violation v = piece(profile, sPrev, previous, s, here, rate, need);
                if (v != null) {
                    return v;
                }
                sPrev = s;
                previous = here;
            }
        }
        return null;
    }

    /** True if {@link #check} finds nothing wrong with the route. */
    public boolean isSafe(HeadingProfile profile) {
        return check(profile) == null;
    }

    /**
     * Checks the piece of route between two samples whose gaps are already known, cutting it in half where the two end
     * gaps cannot prove it safe. {@code rate} is the turn rate along the piece, radians per inch.
     */
    private Violation piece(HeadingProfile profile, double sA, double[] gapsA, double sB, double[] gapsB,
                            double rate, double[] need) {
        // Between the two samples every point of the robot moves along a path of at most (inches driven + corner
        // radius x radians turned), so it is never farther than HALF of that from the nearer end sample. If even
        // after backing off by that much both end gaps are still enough, the whole piece is safe. (This also means
        // both end samples are clear, because half the travel is never negative.)
        double driven = sB - sA;
        double halfTravel = 0.5 * driven * (1.0 + reachIn * rate);
        boolean proven = true;
        for (int c = 0; c < count; c++) {
            if (Math.min(gapsA[c], gapsB[c]) - halfTravel < need[c]) {
                proven = false;
                break;
            }
        }
        if (proven) {
            return null;
        }
        if (driven <= settings.minStepIn) {
            // The piece cannot be cut any finer: all that is left to ask is whether the end sample is clear (the start
            // sample was checked as the end of the piece before).
            for (int c = 0; c < count; c++) {
                if (gapsB[c] < need[c]) {
                    return violation(profile, sB, c, gapsB[c], need[c]);
                }
            }
            return null;
        }
        double sM = 0.5 * (sA + sB);
        double[] gapsM = gapsAt(profile, sM);
        Violation first = piece(profile, sA, gapsA, sM, gapsM, rate, need);
        if (first != null) {
            return first;
        }
        return piece(profile, sM, gapsM, sB, gapsB, rate, need);
    }

    private Violation violation(HeadingProfile profile, double s, int c, double gap, double required) {
        return new Violation(s, profile.poseAt(s), names[c], gap, required + settings.toleranceIn);
    }

    /** The gap to every wall and obstacle for the robot at a distance along the route. */
    private double[] gapsAt(HeadingProfile profile, double s) {
        Footprint f = footprintAt(profile.poseAt(s));
        double[] g = new double[count];
        Rect b = f.bounds();
        g[0] = b.minX - field.minX;
        g[1] = field.maxX - b.maxX;
        g[2] = b.minY - field.minY;
        g[3] = field.maxY - b.maxY;
        for (int k = 0; k < obstacles.size(); k++) {
            g[4 + k] = gapToRect(f, obstacles.get(k));
        }
        return g;
    }

    /**
     * The signed gap between the robot's rectangle and a box, in inches: positive = that much clear space, negative =
     * overlapping by that much. It is the separating-axis test (two convex shapes are apart if some line direction
     * separates them) turned into a number: the best separation over the four edge directions of the two rectangles
     * (field x, field y, and the robot's own two axes). When the shapes are apart this never over-states the true
     * distance, and when they overlap it is exactly how far one must move to come apart, so it is safe to rely on.
     */
    static double gapToRect(Footprint f, Rect r) {
        Rect b = f.bounds();
        // Separation along the field's x and y directions.
        double best = Math.max(Math.max(b.minX - r.maxX, r.minX - b.maxX), Math.max(b.minY - r.maxY, r.minY - b.maxY));
        // Separation along the robot's own two axes: its forward direction and its left direction.
        double c = Math.cos(f.pose().heading);
        double s = Math.sin(f.pose().heading);
        double[][] axes = {{c, s}, {-s, c}};
        double[] rx = {r.minX, r.maxX, r.maxX, r.minX};
        double[] ry = {r.minY, r.minY, r.maxY, r.maxY};
        double[] fx = new double[4];
        double[] fy = new double[4];
        for (int i = 0; i < 4; i++) {
            fx[i] = f.corner(i).x;
            fy[i] = f.corner(i).y;
        }
        for (double[] a : axes) {
            double fMin = Double.POSITIVE_INFINITY;
            double fMax = Double.NEGATIVE_INFINITY;
            double rMin = Double.POSITIVE_INFINITY;
            double rMax = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < 4; i++) {
                double pf = fx[i] * a[0] + fy[i] * a[1];
                fMin = Math.min(fMin, pf);
                fMax = Math.max(fMax, pf);
                double pr = rx[i] * a[0] + ry[i] * a[1];
                rMin = Math.min(rMin, pr);
                rMax = Math.max(rMax, pr);
            }
            best = Math.max(best, Math.max(rMin - fMax, fMin - rMax));
        }
        return best;
    }
}
