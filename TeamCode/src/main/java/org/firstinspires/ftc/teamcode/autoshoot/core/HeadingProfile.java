package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The one rule that says WHICH WAY THE ROBOT FACES at every point of a route it is driving.
 *
 * <p>Why this class exists. A route is a list of points to drive through (the "waypoints"). The position of the robot
 * on that route is easy: it is on the straight lines between the points. But the robot is a 17.8 x 17.8 in rectangle, not a
 * dot (and not even centered on its position: the front edge is 9.77 in from it, the back 8.03 in), and a rectangle takes up more
 * room when it is turned (turned 45 degrees it is about 25 in wide, not 17.8).
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
 *   <li>Optionally the planner gives the turn a <b>window</b>: hold the start heading for the first so many inches and
 *       be done turning for the last so many, with the same evenly spread turn in between. That is how a robot
 *       standing against a wall can drive straight out before it turns, and how a robot that is about to arrive at a
 *       wall can finish turning before it gets there. With no window the turn covers the whole route (rule 3 as
 *       written).</li>
 * </ol>
 *
 * <p>Words used here. <b>Vertex</b>: a corner of the route (the start, or a waypoint that was kept, or a corner added
 * at the edge of the turn window). <b>Distance along the route</b>: how many inches the robot has driven from the
 * start, following the straight legs; the profile is a function of that distance. Everything is in the field frame:
 * inches and radians, 0 = facing +x, counter-clockwise positive.
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
    // What was ASKED for, which an EMPTY profile (no waypoint far enough away) would otherwise lose: the heading the robot
    // is to end with, and the point it is sent to (the last waypoint asked for, or the start if there were none).
    private final double requestedEndHeading;
    private final double holdX;
    private final double holdY;

    private HeadingProfile(double[] xs, double[] ys, double[] along, double[] headings, double requestedEndHeading,
                           double holdX, double holdY) {
        this.xs = xs;
        this.ys = ys;
        this.along = along;
        this.headings = headings;
        this.requestedEndHeading = requestedEndHeading;
        this.holdX = holdX;
        this.holdY = holdY;
    }

    /**
     * Builds the profile of the route {@code start -> waypoints}, ending at {@code endHeadingRad}, with the turn spread
     * over the whole route (the ordinary rule above), using the short-leg limit from
     * {@link BioBuzzConstants.AutoShoot#PATH_MIN_LEG_IN}.
     *
     * @param start         where the robot is now (field inches, radians); the start of the route
     * @param waypoints     the points to drive through, each {@code {x, y}} in inches; the start is NOT in the list
     * @param endHeadingRad the heading to have at the last waypoint (radians)
     */
    public static HeadingProfile build(Pose2 start, List<double[]> waypoints, double endHeadingRad) {
        return build(start, waypoints, endHeadingRad, 0.0, 0.0);
    }

    /**
     * Same, but with a <b>turn window</b>: the robot keeps its start heading for the first {@code holdIn} inches, does
     * the whole turn (spread evenly over the distance) in the middle, and has already reached the final heading for the
     * last {@code settleIn} inches. {@code build(start, waypoints, end, 0, 0)} is the ordinary rule. The planner uses a
     * window to turn only where there is room: for example "hold the heading until the robot is 13 in away from the
     * wall, then turn". This is the version the robot and the checker call, so they cannot disagree.
     *
     * <p>The window is part of the PROFILE, so the two ends of the turn become corners of the route: a corner is
     * inserted where a window edge falls inside a leg (Pedro blends the heading in a straight line between two
     * corners, so the turn can only start or stop at a corner). A window edge within the short-leg limit of an existing
     * corner moves onto that corner instead, so no tiny leg appears.
     *
     * @param holdIn   inches at the start during which the heading does not change (0 or more; clamped to the route)
     * @param settleIn inches at the end during which the heading is already the final one (clamped to what is left)
     */
    public static HeadingProfile build(Pose2 start, List<double[]> waypoints, double endHeadingRad,
                                       double holdIn, double settleIn) {
        return buildWithMinLeg(start, waypoints, endHeadingRad, holdIn, settleIn,
                BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN);
    }

    /**
     * Same, with the short-leg limit given explicitly (inches). Only unit tests use this one, to try a different
     * limit; the robot and the checker use the other versions so they cannot disagree.
     */
    public static HeadingProfile buildWithMinLeg(Pose2 start, List<double[]> waypoints, double endHeadingRad,
                                                 double holdIn, double settleIn, double minLegIn) {
        // The points to visit: the start, then each waypoint that is at least minLegIn past the previous KEPT point.
        // Each entry is {x, y, distance along the route}.
        List<double[]> pts = new ArrayList<double[]>();
        pts.add(new double[]{start.x, start.y, 0.0});
        for (double[] w : waypoints) {
            double[] last = pts.get(pts.size() - 1);
            double leg = Math.hypot(w[0] - last[0], w[1] - last[1]);
            // "leg > 0" also keeps a zero-length leg out when the limit is set to 0.
            if (leg >= minLegIn && leg > 0) {
                pts.add(new double[]{w[0], w[1], last[2] + leg});
            }
        }
        double total = pts.get(pts.size() - 1)[2];
        // The turn window: [turnFrom, turnTo] in inches along the route. With no hold and no settle it is the whole
        // route, which is the ordinary rule.
        double hold = Math.max(0.0, Math.min(total, holdIn));
        double settle = Math.max(0.0, Math.min(total - hold, settleIn));
        double turnFrom = hold;
        double turnTo = total - settle;
        if (pts.size() >= 2) {
            // Make the window edges corners of the route (or snap them onto a corner that is close).
            turnFrom = cornerAt(pts, turnFrom, minLegIn);
            turnTo = cornerAt(pts, turnTo, minLegIn);
            if (turnTo < turnFrom) {
                turnTo = turnFrom;
            }
        }
        int n = pts.size();
        double[] xs = new double[n];
        double[] ys = new double[n];
        double[] along = new double[n];
        double[] headings = new double[n];
        // The shortest signed turn (radians) from the start heading to the end heading. Using this instead of
        // (end - start) means the robot never takes the long way round across the +/-PI wrap-around.
        double turn = Angles.error(start.heading, endHeadingRad);
        double window = turnTo - turnFrom;
        for (int i = 0; i < n; i++) {
            xs[i] = pts.get(i)[0];
            ys[i] = pts.get(i)[1];
            along[i] = pts.get(i)[2];
            // The share of the turn done by this corner: 0 before the window, 1 after it, and in proportion to the
            // distance driven inside it. (A window of zero length is a step: corners before it have 0, the rest 1.)
            double share;
            if (window > 0) {
                share = Math.max(0.0, Math.min(1.0, (along[i] - turnFrom) / window));
            } else {
                share = along[i] >= turnTo ? 1.0 : 0.0;
            }
            headings[i] = start.heading + (total > 0 ? turn * share : 0.0);
        }
        double[] lastAsked = waypoints.isEmpty() ? new double[]{start.x, start.y} : waypoints.get(waypoints.size() - 1);
        return new HeadingProfile(xs, ys, along, headings, endHeadingRad, lastAsked[0], lastAsked[1]);
    }

    /**
     * Makes {@code s} (inches along the route) a corner of the route and returns where the corner really is. If an
     * existing corner is closer than {@code minLegIn}, that corner is used (its distance is returned). Otherwise a new
     * corner is inserted on the straight leg that contains {@code s}.
     */
    private static double cornerAt(List<double[]> pts, double s, double minLegIn) {
        for (int i = 0; i < pts.size(); i++) {
            if (Math.abs(pts.get(i)[2] - s) < minLegIn || pts.get(i)[2] == s) {
                return pts.get(i)[2];
            }
        }
        for (int i = 0; i < pts.size() - 1; i++) {
            double[] a = pts.get(i);
            double[] b = pts.get(i + 1);
            if (s > a[2] && s < b[2]) {
                double t = (s - a[2]) / (b[2] - a[2]);
                pts.add(i + 1, new double[]{a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, s});
                return s;
            }
        }
        return s;
    }

    /**
     * True if there is nothing to follow: no waypoint was far enough from the start.
     *
     * <p>CAREFUL: an empty profile is NOT "the robot stays as it is". {@code PedroDrive} still sends the robot to the point it was asked for
     * and turns it to the heading it was asked for, by holding that pose, and a robot that is next to a wall can swing a corner through it
     * while it turns. {@link #turnInPlaceRad}, {@link #holdX} and {@link #holdY} say what that move is, and {@code RouteSweep} checks it.
     */
    public boolean isEmpty() {
        return xs.length < 2;
    }

    /**
     * The turn the robot makes when the profile is EMPTY: the shortest way round (radians, positive = counter-clockwise) from its start
     * heading to the heading it was asked to end with. 0 for a profile that has a route to follow (that one turns along the route, see
     * {@link #headingAtVertex}).
     */
    public double turnInPlaceRad() {
        return isEmpty() ? Angles.error(headings[0], requestedEndHeading) : 0.0;
    }

    /** Field x, inches, of the point an EMPTY profile sends the robot to: the last waypoint it was asked for (the start if there was none). */
    public double holdX() {
        return holdX;
    }

    /** Field y, inches, of the point an EMPTY profile sends the robot to; see {@link #holdX}. */
    public double holdY() {
        return holdY;
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

    /**
     * The fastest turning anywhere on the route, in degrees per inch driven (0 if the heading never changes). The
     * robot can only turn so fast while it drives, so the planner refuses a turn window that makes this too large.
     */
    public double maxTurnRateDegPerIn() {
        double fastest = 0.0;
        for (int i = 0; i + 1 < xs.length; i++) {
            double turn = Math.abs(headings[i + 1] - headings[i]);
            fastest = Math.max(fastest, Math.toDegrees(turn) / (along[i + 1] - along[i]));
        }
        return fastest;
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
