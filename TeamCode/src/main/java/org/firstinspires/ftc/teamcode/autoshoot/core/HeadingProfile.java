package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The one rule that says WHICH WAY THE ROBOT FACES at every point of a route it is driving.
 *
 * <p>Why this class exists. A route is a list of points to drive through (the "waypoints"). The position of the robot
 * on that route is easy: it is on the straight lines between the points. But the robot is a 17.8 in square, not a
 * dot, and a square takes up more room when it is turned (a square turned 45 degrees is about 25 in wide, not 17.8).
 * So whether a route is safe near a wall depends on the robot's <b>heading</b> (the way it faces) at every point of
 * the route, and the heading is not part of the waypoint list: the drive code decides it. If the code that DRIVES the
 * route and the code that CHECKS the route each worked out the heading in their own way, they could slowly disagree
 * and a route could pass the check but be driven differently. So the rule lives here, once, and both use it:
 * {@code PedroDrive} (to build the path it hands to Pedro Pathing) and {@code RouteSweep} (to test the whole route
 * against the walls). Nothing else may turn a waypoint list into headings.
 *
 * <p>The rule (read from the original {@code PedroDrive.buildPath}, and pinned by {@code HeadingProfileTest}):
 * <ol>
 *   <li>The route starts at the robot's pose and passes through the waypoints in order. A waypoint that is less than
 *       {@link BioBuzzConstants.AutoShoot#PATH_MIN_LEG_IN} (0.5 in) past the previous point is dropped, because Pedro
 *       cannot follow a zero-length line.</li>
 *   <li>The robot makes ONE turn, the shortest way round, from its start heading to the final heading. (Shortest way
 *       round: 170 degrees to -170 degrees is a turn of +20 degrees, never -340.)</li>
 *   <li>That turn is spread out in proportion to the distance driven: after a quarter of the route's length the
 *       robot has done a quarter of the turn. Each straight leg then has a start heading and an end heading, and
 *       Pedro blends between them in a straight line ("linear interpolation").</li>
 * </ol>
 *
 * <p>Words used here. <b>Vertex</b>: a corner of the route (the start, or a waypoint that was kept). <b>Distance
 * along the route</b>: how many inches the robot has driven from the start, following the straight legs; the
 * profile is a function of that distance. Everything is in the field frame: inches and radians, 0 = facing +x,
 * counter-clockwise positive.
 *
 * <p>Example: start at (0, 0) facing 0, waypoints (10, 0) and (30, 0), final heading 90 degrees. The route is 30 in
 * long. At (10, 0) a third of the way, the robot faces 30 degrees; at (30, 0) it faces 90.
 *
 * <p>Immutable and pure Java (no Pedro, no FTC SDK), so a unit test on a PC can pin the rule.
 */
public final class HeadingProfile {
    // The corners of the route and how far along (inches) each one is.
    private final double[] xs;
    private final double[] ys;
    private final double[] along;
    // The heading at each corner, radians. These are NOT wrapped into (-PI, PI]: start + a share of the turn can
    // run past PI, which is fine for Pedro and keeps the blend between two corners a plain straight line.
    private final double[] headings;

    private HeadingProfile(double[] xs, double[] ys, double[] along, double[] headings) {
        this.xs = xs;
        this.ys = ys;
        this.along = along;
        this.headings = headings;
    }

    /**
     * Builds the profile of the route {@code start -> waypoints}, ending at {@code endHeadingRad}, using the short-leg
     * limit from {@link BioBuzzConstants.AutoShoot#PATH_MIN_LEG_IN}. This is the one the robot and the checker call.
     *
     * @param start         where the robot is now (field inches, radians); the start of the route
     * @param waypoints     the points to drive through, each {@code {x, y}} in inches; the start is NOT in the list
     * @param endHeadingRad the heading to have at the last waypoint (radians)
     */
    public static HeadingProfile build(Pose2 start, List<double[]> waypoints, double endHeadingRad) {
        return build(start, waypoints, endHeadingRad, BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN);
    }

    /**
     * Same, with the short-leg limit given explicitly (inches). Only unit tests use this one, to try a different
     * limit; the robot and the checker use the three-argument version so they cannot disagree.
     */
    public static HeadingProfile build(Pose2 start, List<double[]> waypoints, double endHeadingRad, double minLegIn) {
        // The points to visit: the start, then each waypoint that is at least minLegIn past the previous KEPT point.
        List<double[]> pts = new ArrayList<double[]>();
        pts.add(new double[]{start.x, start.y});
        for (double[] w : waypoints) {
            double[] last = pts.get(pts.size() - 1);
            double leg = Math.hypot(w[0] - last[0], w[1] - last[1]);
            // "leg > 0" also keeps a zero-length leg out when the limit is set to 0.
            if (leg >= minLegIn && leg > 0) {
                pts.add(new double[]{w[0], w[1]});
            }
        }
        int n = pts.size();
        double[] xs = new double[n];
        double[] ys = new double[n];
        double[] along = new double[n];
        double[] headings = new double[n];
        for (int i = 0; i < n; i++) {
            xs[i] = pts.get(i)[0];
            ys[i] = pts.get(i)[1];
            if (i > 0) {
                along[i] = along[i - 1] + Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
            }
        }
        // The shortest signed turn (radians) from the start heading to the end heading. Using this instead of
        // (end - start) means the robot never takes the long way round across the +/-PI wrap-around.
        double turn = Angles.error(start.heading, endHeadingRad);
        double total = along[n - 1];
        for (int i = 0; i < n; i++) {
            // The share of the turn done by this corner = the share of the distance driven (nothing to turn
            // over if there is no route at all).
            headings[i] = start.heading + (total > 0 ? turn * (along[i] / total) : 0.0);
        }
        return new HeadingProfile(xs, ys, along, headings);
    }

    /** True if there is nothing to follow: no waypoint was far enough from the start. */
    public boolean isEmpty() {
        return xs.length < 2;
    }

    /** Number of corners, the start included (so the route has {@code vertexCount() - 1} straight legs). */
    public int vertexCount() {
        return xs.length;
    }

    /** Field x of corner {@code i}, in inches. */
    public double x(int i) {
        return xs[i];
    }

    /** Field y of corner {@code i}, in inches. */
    public double y(int i) {
        return ys[i];
    }

    /** Inches driven from the start to corner {@code i}. */
    public double distanceAtVertex(int i) {
        return along[i];
    }

    /** The heading at corner {@code i}, radians; NOT wrapped (see the class note), so use {@link Angles} to compare it. */
    public double headingAtVertex(int i) {
        return headings[i];
    }

    /** The whole route's length, in inches (0 when empty). */
    public double length() {
        return along[along.length - 1];
    }

    /** The heading the route starts with (the robot's own heading when it was planned), radians. */
    public double startHeading() {
        return headings[0];
    }

    /** The heading at the end of the route, radians (not wrapped). */
    public double endHeading() {
        return headings[headings.length - 1];
    }

    /**
     * The robot's pose at a distance along the route: its position on the straight leg and its heading blended
     * between the heading at the two ends of that leg. A distance below 0 or past the end is clamped to the start or
     * the end. The returned heading is wrapped into (-PI, PI] like every {@link Pose2}.
     */
    public Pose2 poseAt(double distanceIn) {
        int last = xs.length - 1;
        if (last == 0) {
            return new Pose2(xs[0], ys[0], headings[0]);
        }
        double s = Math.max(0.0, Math.min(along[last], distanceIn));
        // Find the leg that contains s (the route has only a handful of legs, so a plain scan is fine).
        int i = 0;
        while (i < last - 1 && s > along[i + 1]) {
            i++;
        }
        double legLength = along[i + 1] - along[i];
        double t = legLength > 0 ? (s - along[i]) / legLength : 0.0;
        return new Pose2(xs[i] + (xs[i + 1] - xs[i]) * t,
                ys[i] + (ys[i + 1] - ys[i]) * t,
                headings[i] + (headings[i + 1] - headings[i]) * t);
    }

    /** The heading at a distance along the route, wrapped into (-PI, PI]; see {@link #poseAt}. */
    public double headingAt(double distanceIn) {
        return poseAt(distanceIn).heading;
    }
}
