package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Pins the rule that says which way the robot faces along a driven route ({@link HeadingProfile}).
 *
 * <p>Why it matters: the drive code ({@code PedroDrive}) and the route checker ({@code RouteSweep}) both ask this one
 * class for the heading at every point. If the rule changed quietly, a route could still pass the wall check while the
 * robot turned differently on the field. These tests copy the ORIGINAL rule (the code that used to live inside
 * {@code PedroDrive.buildPath}) into the test, written out separately, and compare.
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Waypoint:</b> a point the route passes through. The start is not one of them.</li>
 *   <li><b>Vertex:</b> a corner of the route: the start, or a waypoint that was kept.</li>
 *   <li><b>Heading:</b> the way the robot faces, in radians, 0 = +x, counter-clockwise positive.</li>
 *   <li><b>Seam:</b> the jump between +180 and -180 degrees. 179 and -179 degrees are only 2 degrees apart.</li>
 * </ul>
 */
public class HeadingProfileTest {
    /** Allowed rounding error when comparing two decimal numbers: one millionth. */
    private static final double EPS = 1e-6;

    private static List<double[]> route(double... xy) {
        List<double[]> out = new ArrayList<double[]>();
        for (int i = 0; i < xy.length; i += 2) {
            out.add(new double[]{xy[i], xy[i + 1]});
        }
        return out;
    }

    /**
     * Situation: start (0, 0) facing 0, waypoints (10, 0) and (30, 0), final heading 90 degrees.
     * Expected: the headings at the three corners are 0, 30 and 90 degrees (a third, then all, of the turn).
     * Why: the turn is spread out in proportion to the distance driven. This is the worked example in the class note.
     */
    @Test
    public void theTurnIsSpreadInProportionToTheDistanceDriven() {
        HeadingProfile p = HeadingProfile.build(Pose2.ofDegrees(0, 0, 0), route(10, 0, 30, 0), Math.toRadians(90));
        assertEquals(3, p.vertexCount());
        assertEquals(0.0, Math.toDegrees(p.headingAtVertex(0)), EPS);
        assertEquals(30.0, Math.toDegrees(p.headingAtVertex(1)), EPS);
        assertEquals(90.0, Math.toDegrees(p.headingAtVertex(2)), EPS);
        assertEquals(30.0, p.length(), EPS);
        assertEquals(10.0, p.distanceAtVertex(1), EPS);
    }

    /**
     * Situation: two legs of very different length, 5 in then 45 in, and a 100 degree turn.
     * Expected: the heading at the middle corner is 10 degrees (a tenth of the turn, because the first leg is a tenth
     *     of the route), NOT 50 degrees.
     * Why: a version that gave every leg an equal share of the turn would fail here. It would turn the robot much too
     *     fast on a short first leg, which is exactly where the robot is near the wall it started against.
     */
    @Test
    public void aShortFirstLegGetsOnlyItsShareOfTheTurnNotAnEqualShare() {
        HeadingProfile p = HeadingProfile.build(Pose2.ofDegrees(0, 0, 0), route(5, 0, 50, 0), Math.toRadians(100));
        assertEquals(10.0, Math.toDegrees(p.headingAtVertex(1)), EPS);
        assertEquals(100.0, Math.toDegrees(p.headingAtVertex(2)), EPS);
    }

    /**
     * Situation: start heading 170 degrees, final heading -170 degrees (the other side of the seam), a 40 in route.
     * Expected: the robot turns +20 degrees (counter-clockwise), the heading half way is 180 degrees, and the turn
     *     the profile ends with is +20 degrees away from where it started.
     * Why: the long way round would be a 340 degree spin. This is the "shortest turn" rule.
     */
    @Test
    public void theTurnTakesTheShortWayRoundAcrossTheSeam() {
        HeadingProfile p = HeadingProfile.build(Pose2.ofDegrees(0, 0, 170), route(40, 0), Math.toRadians(-170));
        assertEquals(20.0, Math.toDegrees(Angles.error(p.startHeading(), p.endHeading())), EPS);
        assertEquals(180.0, Math.abs(Math.toDegrees(p.headingAt(20.0))), EPS);
        // And the other direction round the seam is a turn of -20 degrees.
        HeadingProfile q = HeadingProfile.build(Pose2.ofDegrees(0, 0, -170), route(40, 0), Math.toRadians(170));
        assertEquals(-20.0, Math.toDegrees(Angles.error(q.startHeading(), q.endHeading())), EPS);
    }

    /**
     * Situation: the original algorithm (copied below), run on 300 random routes with random start and end headings,
     *     some with waypoints closer than the 0.5 in short-leg limit.
     * Expected: the same corners are kept and every corner has the same heading, to a millionth of a radian.
     * Why: this is the guard against drift. The original rule is written out here independently of the class.
     */
    @Test
    public void matchesTheOriginalPedroDriveRuleOnRandomRoutes() {
        Random rng = new Random(5);
        for (int trial = 0; trial < 300; trial++) {
            Pose2 start = new Pose2(rng.nextDouble() * BioBuzzConstants.Field.SIZE_IN, rng.nextDouble() * BioBuzzConstants.Field.SIZE_IN, (rng.nextDouble() - 0.5) * 7.0);
            List<double[]> wps = new ArrayList<double[]>();
            double x = start.x;
            double y = start.y;
            int n = 1 + rng.nextInt(5);
            for (int i = 0; i < n; i++) {
                // Mix long legs and tiny legs (0 to 0.8 in) so the short-leg rule is exercised.
                double len = rng.nextBoolean() ? rng.nextDouble() * 60 : rng.nextDouble() * 0.8;
                double ang = rng.nextDouble() * 2 * Math.PI;
                x += len * Math.cos(ang);
                y += len * Math.sin(ang);
                wps.add(new double[]{x, y});
            }
            double endHeading = (rng.nextDouble() - 0.5) * 7.0;
            double[][] expected = originalRule(start, wps, endHeading);       // rows: x, y, heading
            HeadingProfile p = HeadingProfile.build(start, wps, endHeading);
            if (expected == null) {
                assertTrue("trial " + trial, p.isEmpty());
                continue;
            }
            assertEquals("trial " + trial, expected.length, p.vertexCount());
            for (int i = 0; i < expected.length; i++) {
                assertEquals(expected[i][0], p.x(i), EPS);
                assertEquals(expected[i][1], p.y(i), EPS);
                assertEquals("trial " + trial + " corner " + i, expected[i][2], p.headingAtVertex(i), EPS);
            }
        }
    }

    /**
     * The original rule, exactly as {@code PedroDrive.buildPath} had it before it was moved: keep a waypoint only if
     * it is at least the short-leg limit past the previous kept point; give each kept point the start heading plus
     * the shortest turn times (distance so far / total distance). Returns {x, y, heading} per corner, or null if
     * nothing was kept.
     */
    private static double[][] originalRule(Pose2 start, List<double[]> waypoints, double endHeading) {
        List<double[]> pts = new ArrayList<double[]>();
        pts.add(new double[]{start.x, start.y});
        for (double[] w : waypoints) {
            double[] last = pts.get(pts.size() - 1);
            if (Math.hypot(w[0] - last[0], w[1] - last[1]) >= 0.5) {
                pts.add(new double[]{w[0], w[1]});
            }
        }
        if (pts.size() < 2) {
            return null;
        }
        double total = 0.0;
        double[] cumulative = new double[pts.size()];
        for (int i = 1; i < pts.size(); i++) {
            total += Math.hypot(pts.get(i)[0] - pts.get(i - 1)[0], pts.get(i)[1] - pts.get(i - 1)[1]);
            cumulative[i] = total;
        }
        double turn = Angles.error(start.heading, endHeading);
        double[][] out = new double[pts.size()][];
        for (int i = 0; i < pts.size(); i++) {
            out[i] = new double[]{pts.get(i)[0], pts.get(i)[1], start.heading + turn * (cumulative[i] / total)};
        }
        return out;
    }

    /**
     * Situation: waypoints 0.3 in, 0.45 in and 0.6 in from the start along a line, then one 20 in away.
     * Expected: the 0.3 and 0.45 in points are dropped (under the 0.5 in limit); the 0.6 in point is kept; a leg of
     *     exactly the limit is kept (the test is "at least"); distances are measured from the last KEPT corner.
     * Why: PedroDrive used to drop these because Pedro cannot follow a zero-length line, and the checker has to
     *     check the path that is really driven, so it must drop the very same points.
     */
    @Test
    public void shortLegsAreDroppedMeasuredFromTheLastKeptCorner() {
        double limit = BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN;
        HeadingProfile p = HeadingProfile.build(Pose2.ofDegrees(0, 0, 0),
                route(0.3, 0, 0.45, 0, 0.6, 0, 20.6, 0), 0.0);
        assertEquals(3, p.vertexCount());                 // start, the 0.6 in point, the 20.6 in point
        assertEquals(0.6, p.x(1), EPS);
        assertEquals(20.6, p.x(2), EPS);
        // A waypoint exactly the limit away is kept.
        HeadingProfile q = HeadingProfile.build(Pose2.ofDegrees(0, 0, 0), route(limit, 0), 0.0);
        assertEquals(2, q.vertexCount());
        // And one a hair short of it is not.
        HeadingProfile r = HeadingProfile.build(Pose2.ofDegrees(0, 0, 0), route(limit - 0.01, 0), 0.0);
        assertTrue(r.isEmpty());
    }

    /**
     * Situation: an empty waypoint list, and a list whose only waypoint sits on the start.
     * Expected: the profile is empty (nothing to follow), has no legs, and its length is zero; asking for a pose still
     *     answers with the start pose.
     * Why: PedroDrive then holds the pose instead of building a path.
     */
    @Test
    public void aRouteWithNothingFarEnoughAwayIsEmpty() {
        Pose2 start = Pose2.ofDegrees(10, 20, 30);
        HeadingProfile none = HeadingProfile.build(start, Collections.<double[]>emptyList(), 1.0);
        assertTrue(none.isEmpty());
        assertEquals(0.0, none.length(), EPS);
        HeadingProfile onTop = HeadingProfile.build(start, route(10, 20), 1.0);
        assertTrue(onTop.isEmpty());
        Pose2 at = onTop.poseAt(5.0);
        assertEquals(10.0, at.x, EPS);
        assertEquals(20.0, at.y, EPS);
        assertEquals(30.0, at.headingDegrees(), EPS);
        assertFalse(HeadingProfile.build(start, route(30, 20), 1.0).isEmpty());
    }

    /**
     * Situation: the robot already faces the final heading.
     * Expected: the heading is the same at every corner and everywhere in between.
     * Why: no turn means no spinning along the route; the footprint stays the shape it started with.
     */
    @Test
    public void noTurnWhenTheRobotAlreadyFacesTheEndHeading() {
        HeadingProfile p = HeadingProfile.build(Pose2.ofDegrees(5, 5, -90), route(5, 40, 50, 40), Math.toRadians(-90));
        for (double s = 0.0; s <= p.length(); s += 3.7) {
            assertEquals(-90.0, p.poseAt(s).headingDegrees(), EPS);
        }
    }

    /**
     * Situation: a two-leg route (10 in east, then 10 in north) and questions at 5 in, 15 in, below 0 and past the end.
     * Expected: 5 in is half way along the first leg; 15 in is half way along the second; the position and the
     *     heading are blended along each leg; below 0 gives the start and past the end gives the end.
     * Why: the checker asks for the pose at arbitrary distances, and the answer has to be the point Pedro's straight
     *     legs really pass.
     */
    @Test
    public void poseAtBlendsPositionAndHeadingAlongEachLegAndClamps() {
        HeadingProfile p = HeadingProfile.build(Pose2.ofDegrees(0, 0, 0), route(10, 0, 10, 10), Math.toRadians(40));
        Pose2 a = p.poseAt(5.0);
        assertEquals(5.0, a.x, EPS);
        assertEquals(0.0, a.y, EPS);
        assertEquals(10.0, a.headingDegrees(), EPS);
        Pose2 b = p.poseAt(15.0);
        assertEquals(10.0, b.x, EPS);
        assertEquals(5.0, b.y, EPS);
        assertEquals(30.0, b.headingDegrees(), EPS);
        assertEquals(0.0, p.poseAt(-3.0).x, EPS);
        assertEquals(40.0, p.poseAt(99.0).headingDegrees(), EPS);
        assertEquals(10.0, p.poseAt(99.0).y, EPS);
        assertEquals(p.headingAt(7.0), p.poseAt(7.0).heading, EPS);
    }

    /**
     * Situation: the same route built with the short-leg limit given explicitly as the constant, and with 3 in.
     * Expected: the plain 3-argument version equals the one given the constant; a bigger limit drops more corners.
     * Why: the robot and the checker call the 3-argument version, so the limit really comes from BioBuzzConstants.
     */
    @Test
    public void theDefaultShortLegLimitIsTheConstant() {
        List<double[]> wps = route(1.0, 0, 2.0, 0, 20.0, 0);
        Pose2 start = Pose2.ofDegrees(0, 0, 0);
        HeadingProfile byDefault = HeadingProfile.build(start, wps, 1.0);
        HeadingProfile explicit = HeadingProfile.buildWithMinLeg(start, wps, 1.0, 0.0, 0.0,
                BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN);
        assertEquals(explicit.vertexCount(), byDefault.vertexCount());
        HeadingProfile coarse = HeadingProfile.buildWithMinLeg(start, wps, 1.0, 0.0, 0.0, 3.0);
        assertTrue(coarse.vertexCount() < byDefault.vertexCount());
        assertEquals(Arrays.asList(0.0, 20.0), Arrays.asList(coarse.x(0), coarse.x(1)));
    }

    // ---------------------------------------------------------------- the turn window

    /**
     * Situation: a straight 100 in route east, a 90 degree turn, hold the start heading for 30 in and be finished
     *     turning for the last 20 in.
     * Expected: corners are added at 30 in and 80 in; the heading is 0 up to 30 in, climbs evenly to 90 at 80 in
     *     (45 degrees at 55 in), and stays 90 to the end. The positions are still the straight line.
     * Why: this is the "drive straight out of the wall, then turn, then arrive already facing the right way" rule the
     *     planner uses to keep a turning robot away from a wall.
     */
    @Test
    public void aTurnWindowHoldsTheStartHeadingTurnsEvenlyThenHoldsTheEndHeading() {
        HeadingProfile p = HeadingProfile.build(Pose2.ofDegrees(0, 0, 0), route(100, 0), Math.toRadians(90), 30.0, 20.0);
        assertEquals(4, p.vertexCount());
        assertEquals(30.0, p.distanceAtVertex(1), EPS);
        assertEquals(80.0, p.distanceAtVertex(2), EPS);
        assertEquals(0.0, Math.toDegrees(p.headingAtVertex(1)), EPS);
        assertEquals(90.0, Math.toDegrees(p.headingAtVertex(2)), EPS);
        assertEquals(0.0, p.headingAt(10.0), EPS);
        assertEquals(45.0, Math.toDegrees(p.headingAt(55.0)), EPS);
        assertEquals(90.0, Math.toDegrees(p.headingAt(95.0)), EPS);
        for (double s = 0.0; s <= 100.0; s += 7.0) {
            assertEquals(s, p.poseAt(s).x, EPS);
            assertEquals(0.0, p.poseAt(s).y, EPS);
        }
    }

    /**
     * Situation: the same route with no window (hold 0, settle 0), and with the 3-argument version.
     * Expected: both equal the ordinary rule (27 degrees at 30 in), and the windowed version above is different there
     *     (0 degrees at 30 in).
     * Why: the window must be opt-in. Everything that does not ask for one keeps the pinned original behaviour.
     */
    @Test
    public void noWindowIsTheOrdinaryRule() {
        Pose2 start = Pose2.ofDegrees(0, 0, 0);
        HeadingProfile plain = HeadingProfile.build(start, route(100, 0), Math.toRadians(90));
        HeadingProfile zero = HeadingProfile.build(start, route(100, 0), Math.toRadians(90), 0.0, 0.0);
        HeadingProfile windowed = HeadingProfile.build(start, route(100, 0), Math.toRadians(90), 30.0, 0.0);
        assertEquals(27.0, Math.toDegrees(plain.headingAt(30.0)), EPS);
        assertEquals(27.0, Math.toDegrees(zero.headingAt(30.0)), EPS);
        assertEquals(2, zero.vertexCount());
        assertEquals(0.0, Math.toDegrees(windowed.headingAt(30.0)), EPS);
    }

    /**
     * Situation: a route with a corner at 30 in. A window edge asked for at 30.3 in (within the 0.5 in short-leg limit
     *     of the corner), and another asked for at 31 in.
     * Expected: 30.3 moves onto the corner (no new corner, so no 0.3 in leg); 31 gets its own new corner.
     * Why: Pedro cannot follow a tiny leg, so a window edge must never create one.
     */
    @Test
    public void aWindowEdgeNearACornerMovesOntoTheCorner() {
        Pose2 start = Pose2.ofDegrees(0, 0, 0);
        HeadingProfile snapped = HeadingProfile.build(start, route(30, 0, 100, 0), Math.toRadians(90), 30.3, 0.0);
        assertEquals(3, snapped.vertexCount());
        assertEquals(0.0, Math.toDegrees(snapped.headingAtVertex(1)), EPS);
        HeadingProfile own = HeadingProfile.build(start, route(30, 0, 100, 0), Math.toRadians(90), 31.0, 0.0);
        assertEquals(4, own.vertexCount());
        assertEquals(31.0, own.distanceAtVertex(2), EPS);
    }

    /**
     * Situation: a hold and a settle that together are longer than the 100 in route (90 + 90), and a hold of 500 in.
     * Expected: the settle is cut back to what is left (10 in), leaving a window of zero length: the heading steps
     *     from the start heading to the end heading at 90 in. A hold longer than the route puts the step at the very end.
     * Why: nonsense windows must not crash or produce a heading outside the start..end range.
     */
    @Test
    public void aWindowLongerThanTheRouteIsClamped() {
        Pose2 start = Pose2.ofDegrees(0, 0, 0);
        HeadingProfile p = HeadingProfile.build(start, route(100, 0), Math.toRadians(90), 90.0, 90.0);
        assertEquals(3, p.vertexCount());
        assertEquals(0.0, Math.toDegrees(p.headingAtVertex(0)), EPS);
        assertEquals(90.0, Math.toDegrees(p.headingAtVertex(1)), EPS);
        assertEquals(90.0, Math.toDegrees(p.headingAtVertex(2)), EPS);
        HeadingProfile q = HeadingProfile.build(start, route(100, 0), Math.toRadians(90), 500.0, 0.0);
        assertEquals(2, q.vertexCount());
        assertEquals(0.0, Math.toDegrees(q.headingAtVertex(0)), EPS);
        assertEquals(90.0, Math.toDegrees(q.headingAtVertex(1)), EPS);
    }

    /**
     * Situation: a 100 in straight route turning 90 degrees. First with the turn spread over the whole route, then with
     *     the turn squeezed into a window that leaves 20 in for it (hold 50 in, settle 30 in), then a route that does not
     *     turn at all.
     * Expected: the fastest turning is 0.9 degrees per inch for the even turn, 4.5 degrees per inch for the squeezed one
     *     (90 degrees over 20 in), and 0 when the heading never changes.
     * Why: the planner refuses a window that asks for more turning per inch than the robot can manage, and it does so by
     *     asking this number. A version that averaged over the whole route would report 0.9 for the squeezed turn and
     *     wrongly let it through.
     */
    @Test
    public void theFastestTurnRateIsReadFromTheSteepestLeg() {
        Pose2 start = Pose2.ofDegrees(0, 0, 0);
        double end = Math.toRadians(90);
        assertEquals(0.9, HeadingProfile.build(start, route(100, 0), end).maxTurnRateDegPerIn(), EPS);
        assertEquals(4.5, HeadingProfile.build(start, route(100, 0), end, 50.0, 30.0).maxTurnRateDegPerIn(), EPS);
        assertEquals(0.0, HeadingProfile.build(start, route(100, 0), 0.0).maxTurnRateDegPerIn(), EPS);
    }

    /**
     * Situation: 300 random routes with random windows.
     * Expected: the windowed route is the same polyline (same length, and the position at every distance is the same as
     *     without a window); no leg is shorter than the short-leg limit; every heading lies between the start and the
     *     end heading (it never turns more than the one shortest turn).
     * Why: a window changes WHEN the robot turns, never where it drives or how far it turns.
     */
    @Test
    public void aWindowOnlyChangesWhenTheRobotTurns() {
        Random rng = new Random(9);
        double limit = BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN;
        for (int trial = 0; trial < 300; trial++) {
            Pose2 start = new Pose2(rng.nextDouble() * 100, rng.nextDouble() * 100, (rng.nextDouble() - 0.5) * 6.0);
            List<double[]> wps = new ArrayList<double[]>();
            double x = start.x;
            double y = start.y;
            int n = 1 + rng.nextInt(4);
            for (int i = 0; i < n; i++) {
                double len = 3 + rng.nextDouble() * 40;
                double ang = rng.nextDouble() * 2 * Math.PI;
                x += len * Math.cos(ang);
                y += len * Math.sin(ang);
                wps.add(new double[]{x, y});
            }
            double end = (rng.nextDouble() - 0.5) * 6.0;
            HeadingProfile plain = HeadingProfile.build(start, wps, end);
            double hold = rng.nextDouble() * plain.length();
            double settle = rng.nextDouble() * plain.length();
            HeadingProfile w = HeadingProfile.build(start, wps, end, hold, settle);
            assertEquals(plain.length(), w.length(), EPS);
            double turn = Angles.error(start.heading, end);
            for (int i = 0; i + 1 < w.vertexCount(); i++) {
                assertTrue("trial " + trial + " leg " + i, w.distanceAtVertex(i + 1) - w.distanceAtVertex(i) >= limit - EPS);
            }
            for (int i = 0; i < w.vertexCount(); i++) {
                double s = w.distanceAtVertex(i);
                assertEquals(plain.poseAt(s).x, w.x(i), EPS);
                assertEquals(plain.poseAt(s).y, w.y(i), EPS);
                double done = w.headingAtVertex(i) - start.heading;       // how much of the turn is done at this corner
                double lo = Math.min(0.0, turn);
                double hi = Math.max(0.0, turn);
                assertTrue("trial " + trial, done >= lo - EPS && done <= hi + EPS);
            }
        }
    }
}
