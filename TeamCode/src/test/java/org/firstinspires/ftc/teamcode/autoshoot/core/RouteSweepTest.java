package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Tests the swept-footprint check {@link RouteSweep}: does the robot's REAL turning rectangle stay off the walls, the
 * HIVE frame and the FLOWER plates along a whole route, not just at the two ends?
 *
 * <p>How it works: no robot. Each test builds a small route (a start pose, waypoints and a final heading), turns it
 * into the robot's headings with the same {@link HeadingProfile} the drive uses, and asks the checker. Where the answer
 * must be "unsafe", a negative twin of the same route that is safe shows the failure comes from the thing being
 * tested (the turn, the wall) and not from something else. {@link BruteForceFootprint} is a slow, independent referee.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*RouteSweepTest"} to run only this
 * class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Gap:</b> the empty space between the robot's rectangle and something, in inches. Zero = touching.</li>
 *   <li><b>Clearance / margin:</b> the gap the route must keep: the wall clearance for walls, the margin for obstacles.</li>
 *   <li><b>Footprint:</b> the floor space the robot covers: a 17.8 in square, turned to the robot's heading.</li>
 *   <li><b>Sample:</b> one place along the route where the checker builds the footprint and measures the gaps.</li>
 * </ul>
 */
public class RouteSweepTest {
    private static final double EPS = 1e-6;
    /**
     * These tests are about the sweep's maths, so they use a plain 17.8 in square CENTERED on the robot's position (half of
     * it is 8.9 in) to keep their numbers easy to read. The real robot is not centered: its edges are 9.77 / 8.03 / 8.90 /
     * 8.90 in from its center of rotation, and that robot is tested in AsymmetricRobotTest.
     */
    private static final double SIZE = BioBuzzConstants.Robot.LENGTH_IN;
    private static final double HALF = SIZE / 2.0;
    private static final RobotShape SQUARE = RobotShape.centered(SIZE, SIZE);
    private static final double CLEAR = BioBuzzConstants.AutoShoot.WALL_CLEARANCE_IN;
    private static final double TOL = BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN;
    /** A plain 100 x 100 in test field. */
    private static final Rect BOX = new Rect(0, 100, 0, 100);

    private static RouteSweep sweep(Rect field, double wallClear, List<Rect> obstacles, double margin) {
        return new RouteSweep(field, wallClear, obstacles, margin, SQUARE, RouteSweep.Settings.fromConstants());
    }

    private static RouteSweep plain() {
        return sweep(BOX, CLEAR, Collections.<Rect>emptyList(), 0.0);
    }

    private static HeadingProfile route(Pose2 start, double endHeadingDeg, double... xy) {
        List<double[]> wps = new ArrayList<double[]>();
        for (int i = 0; i < xy.length; i += 2) {
            wps.add(new double[]{xy[i], xy[i + 1]});
        }
        return HeadingProfile.build(start, wps, Math.toRadians(endHeadingDeg));
    }

    // ---------------------------------------------------------------- walls

    /**
     * Situation: the robot drives 40 in down the middle of the field at the same heading.
     * Expected: safe.
     * Why: the check must not cry wolf. A route nowhere near anything has nothing to report.
     */
    @Test
    public void aRouteThroughTheMiddleOfTheFieldIsSafe() {
        assertNull(plain().check(route(Pose2.ofDegrees(30, 50, 0), 0, 70, 50)));
    }

    /**
     * Situation: the robot starts 0.25 in off the audience wall (y = 0), centre 9.15 in out, facing -90, and drives 40
     *     in along the wall while turning 25 degrees to face -65.
     * Expected: the check finds a violation of the wall y = 0 early in the drive, where the first corner swings into
     *     the wall. The independent referee agrees that the true wall is clipped by more than 2 in.
     * Why: this is THE bug. A point 9.15 in from the wall is fine for a robot facing an axis, but a 17.8 in square turned
     *     25 degrees needs 11.8 in from its centre to the wall. A route planned for a dot clips here.
     * Negative twin: the same drive WITHOUT the turn (final heading -90) is safe, and so is the same 25 degree turn done
     *     30 in away from the wall.
     */
    @Test
    public void turningWhileDrivingAlongAWallSwingsACornerThroughIt() {
        Pose2 start = Pose2.ofDegrees(20, HALF + CLEAR, -90);
        HeadingProfile turning = route(start, -65, 60, HALF + CLEAR);
        RouteSweep.Violation v = plain().check(turning);
        assertNotNull("a 25 degree turn along the wall must be caught", v);
        assertTrue(v.what, v.what.startsWith("wall y = 0.0"));
        assertTrue("caught early, at " + v.distanceIn, v.distanceIn < 20.0);
        assertTrue(v.gapIn < v.requiredIn);
        String clip = BruteForceFootprint.firstClip(turning, SQUARE, BOX, Collections.<Rect>emptyList(), 2.0);
        assertNotNull("the referee must see a real clip too", clip);

        // Negative twins.
        assertNull(plain().check(route(start, -90, 60, HALF + CLEAR)));
        assertNull(plain().check(route(Pose2.ofDegrees(20, 30, -90), -65, 60, 30)));
    }

    /**
     * Situation: the robot sits in the corner of the field (centre 9.15 in from both walls), facing 0, and its route
     *     is a 0.6 in nudge while it turns 90 degrees (to face +90, the same square outline as before).
     * Expected: a violation, even though the first and last footprints are the same square and both fit.
     * Why: halfway through the turn the square is at 45 degrees and its corners reach 12.6 in from its centre. A check
     *     that only looked at the end poses, or that treated the robot as a circle of 8.9 in, would miss it. Turning in
     *     place next to a wall sweeps the corners.
     * Negative twin: the same turn done with the robot centre 14 in from both walls is safe.
     */
    @Test
    public void turningInAPlaceNextToAWallSweepsTheCorners() {
        double c = HALF + CLEAR;
        RouteSweep.Violation v = plain().check(route(Pose2.ofDegrees(c, c, 0), 90, c + 0.6, c));
        assertNotNull(v);
        assertTrue(v.what, v.what.startsWith("wall"));
        assertNull(plain().check(route(Pose2.ofDegrees(14, 14, 0), 90, 14.6, 14)));
    }

    /**
     * Situation: the robot starts touching the audience wall (centre 8.9 in out, gap 0) facing -90.
     *     (a) It drives 40 in along the wall, staying at gap 0.
     *     (b) It drives 40 in while slowly sinking 0.9 in toward the wall.
     * Expected: (a) is safe, (b) is a violation of the wall.
     * Why: a robot that starts the match touching a wall is already closer than the 0.25 in clearance, so the clearance
     *     cannot be demanded. The rule is "never squeeze harder than at the start": touching is fine, pushing through
     *     is not.
     */
    @Test
    public void aRobotThatStartsTouchingAWallMayStayTouchingButNotPushThrough() {
        Pose2 start = Pose2.ofDegrees(20, HALF, -90);
        assertNull(plain().check(route(start, -90, 60, HALF)));
        RouteSweep.Violation v = plain().check(route(start, -90, 60, HALF - 0.9));
        assertNotNull(v);
        assertTrue(v.what, v.what.startsWith("wall y = 0.0"));
    }

    /**
     * Situation: the robot starts CLEAR of the wall (20 in out) and drives to 0.1 in from it (inside the 0.25 in
     *     clearance), and also to 0.25 in from it (exactly the clearance).
     * Expected: 0.1 in is a violation; 0.25 in is safe; 0.21 in is safe (inside the 0.05 in tolerance) and 0.19 in is
     *     not.
     * Why: the start exception must not become a loophole. A route that begins clear has to keep the full clearance,
     *     less only the forgiven rounding tolerance.
     */
    @Test
    public void aRobotThatStartsClearMustKeepTheWallClearance() {
        Pose2 start = Pose2.ofDegrees(20, 20, -90);
        assertNotNull(plain().check(route(start, -90, 60, HALF + 0.1)));
        assertNull(plain().check(route(start, -90, 60, HALF + CLEAR)));
        assertNull(plain().check(route(start, -90, 60, HALF + CLEAR - 0.04)));
        assertNotNull(plain().check(route(start, -90, 60, HALF + CLEAR - 0.06)));
    }

    /**
     * Situation: a route whose ONLY problem is its final pose: it drives in the middle of the field and the last
     *     waypoint puts the robot 4 in from the wall (the robot is 8.9 in half-size).
     * Expected: a violation reported at the very end of the route (distance = the route's length).
     * Why: the last pose of a route must always be included in the check.
     */
    @Test
    public void theFinalPoseIsAlwaysChecked() {
        HeadingProfile p = route(Pose2.ofDegrees(50, 50, 0), 0, 50, 20, 50, 4);
        RouteSweep.Violation v = plain().check(p);
        assertNotNull(v);
        assertTrue("the violation starts at the end: " + v, v.distanceIn > p.length() - 6.0);
    }

    /**
     * Situation: the check is asked about an empty route (nothing to drive).
     * Expected: null (safe).
     * Why: the robot does not move, so it cannot hit anything new.
     */
    @Test
    public void anEmptyRouteIsSafe() {
        HeadingProfile none = HeadingProfile.build(Pose2.ofDegrees(5, 5, 0), Collections.<double[]>emptyList(), 0.0);
        assertNull(plain().check(none));
    }

    // ---------------------------------------------------------------- obstacles

    /**
     * Situation: a 20 x 20 in box (like the HIVE frame) in the middle of the field. The robot drives past it with its
     *     rectangle 1.1 in from the box (centre line 30 in, box edge at 40 in, half-size 8.9 in), (a) facing the same way
     *     the whole time, (b) turning 45 degrees on the way.
     * Expected: (a) safe, (b) a violation naming the obstacle.
     * Why: turned, the square's corners reach 12.6 in from its centre, which is 2.6 in into the box. Points 3.6 in apart
     *     would be missed by a check that only looked at a circle of half the width.
     */
    @Test
    public void turningPastAnObstacleSwingsACornerIntoIt() {
        List<Rect> box = Collections.singletonList(new Rect(40, 60, 40, 60));
        RouteSweep s = sweep(BOX, CLEAR, box, 0.0);
        Pose2 start = Pose2.ofDegrees(12, 30, 0);
        assertNull(s.check(route(start, 0, 88, 30)));
        RouteSweep.Violation v = s.check(route(start, 45, 88, 30));
        assertNotNull(v);
        assertTrue(v.what, v.what.startsWith("obstacle 0"));
        assertNotNull(BruteForceFootprint.firstClip(route(start, 45, 88, 30), SQUARE, BOX, box, TOL));
    }

    /**
     * Situation: the same box with a 1.5 in margin, and the robot passing with a gap of only 1.1 in without turning.
     * Expected: a violation; with the gap widened to 2 in, safe.
     * Why: obstacles carry a margin the way walls carry a clearance. (Standing poses keep the same margin.)
     */
    @Test
    public void theObstacleMarginIsKept() {
        List<Rect> box = Collections.singletonList(new Rect(40, 60, 40, 60));
        RouteSweep s = sweep(BOX, CLEAR, box, 1.5);
        assertNotNull(s.check(route(Pose2.ofDegrees(12, 30, 0), 0, 88, 30)));        // gap 1.1 in
        assertNull(s.check(route(Pose2.ofDegrees(12, 29, 0), 0, 88, 29)));           // gap 2.1 in
    }

    /**
     * Situation: the real field. A robot drives up the left wall from (9.15, 20) to (9.15, 80), straight through where
     *     the left-wall FLOWER plate sits (its centre is (2.4, 48), 5 in deep and 6 in long), and a second robot does
     *     the same 10 in further out.
     * Expected: the first is a violation naming an obstacle (the plate); the second, whose side edge stays beyond the
     *     plate's margin, is safe.
     * Why: the FLOWER plates are part of the check with the real numbers, not only the HIVE frame.
     */
    @Test
    public void aFlowerPlateIsAnObstacle() {
        FieldModel field = new FieldModel();
        RouteSweep s = sweep(field.fieldRect(), CLEAR, field.hardObstacles(), BioBuzzConstants.AutoShoot.KEEPOUT_MARGIN_IN);
        RouteSweep.Violation v = s.check(route(Pose2.ofDegrees(9.15, 20, 90), 90, 9.15, 80));
        assertNotNull(v);
        assertTrue(v.what, v.what.startsWith("obstacle"));
        assertNull(s.check(route(Pose2.ofDegrees(20, 20, 90), 90, 20, 80)));
    }

    /**
     * Situation: the real HIVE frame (47.3 to 96.7 in across, 52.5 to 91.5 in deep). A robot drives straight up beside
     *     it with its center 36 in from the red wall, so its right side is 2.4 in from the frame: (a) facing +90 degrees
     *     the whole way, (b) turning from 90 to 45 degrees on the way, which at the HIVE's middle is 67.5 degrees.
     * Expected: (a) safe (2.4 in is more than the 1.5 in margin); (b) a violation naming the obstacle, because a square
     *     turned 22.5 degrees off an axis is 11.6 in from center to side, not 8.9 in, which puts it inside the margin.
     *     The slow referee agrees that (b) comes closer to the frame than the margin allows.
     * Why: the HIVE frame is what the robot most often drives around. The check must use its real size and the same
     *     margin that standing poses keep, and a corner swinging toward it counts.
     */
    @Test
    public void turningPastTheRealHiveFrameSwingsACornerIntoItsMargin() {
        FieldModel field = new FieldModel();
        double margin = BioBuzzConstants.AutoShoot.KEEPOUT_MARGIN_IN;
        List<Rect> hive = Collections.singletonList(field.hiveFrame());
        RouteSweep s = sweep(field.fieldRect(), CLEAR, hive, margin);
        Pose2 start = Pose2.ofDegrees(36, 30, 90);
        assertNull(s.check(route(start, 90, 36, 110)));
        RouteSweep.Violation v = s.check(route(start, 45, 36, 110));
        assertNotNull(v);
        assertTrue(v.what, v.what.startsWith("obstacle 0"));
        // The referee, asked to keep the same margin (it shrinks the box by the tolerance it is given, so grow it).
        Rect grown = field.hiveFrame().inflate(margin);
        assertNotNull(BruteForceFootprint.firstClip(route(start, 45, 36, 110), SQUARE, field.fieldRect(),
                Collections.singletonList(grown), TOL));
        assertNull(BruteForceFootprint.firstClip(route(start, 90, 36, 110), SQUARE, field.fieldRect(),
                Collections.singletonList(grown), TOL));
    }

    /**
     * Situation: a tiny 2 x 2 in robot, a very coarse look (every 5 in), and a wall only 0.1 in thick across its path.
     *     The thin wall falls BETWEEN two of the coarse samples.
     * Expected: the check still finds it. A wall 0.1 in thick somewhere else (not across the path) is not reported.
     * Why: a check that only trusted its samples could let a robot tunnel through a thin obstacle. This one cuts a piece
     *     of route in half until it can prove the piece safe, so it cannot.
     */
    @Test
    public void aThinObstacleCannotSlipBetweenSamples() {
        RouteSweep.Settings coarse = RouteSweep.Settings.fromConstants();
        coarse.stepIn = 5.0;
        coarse.stepDeg = 45.0;
        Rect thin = new Rect(52.0, 52.1, 0, 100);
        RouteSweep s = new RouteSweep(BOX, 0.0, Collections.singletonList(thin), 0.0, RobotShape.centered(2.0, 2.0), coarse);
        HeadingProfile p = route(Pose2.ofDegrees(10, 50, 0), 0, 90, 50);
        RouteSweep.Violation v = s.check(p);
        assertNotNull("must catch a 0.1 in wall between samples", v);
        assertTrue(v.what, v.what.startsWith("obstacle 0"));
        // A wall that is 0.1 in thick but does not span the robot's path (it ends at y = 40) is not a problem.
        RouteSweep elsewhere = new RouteSweep(BOX, 0.0, Collections.singletonList(new Rect(52.0, 52.1, 0, 40)), 0.0,
                RobotShape.centered(2.0, 2.0), coarse);
        assertNull(elsewhere.check(p));
    }

    /**
     * Situation: the violation record of a route that sinks into the wall.
     * Expected: it carries the distance along the route, a pose on the route, the name of the wall, and gap numbers
     *     (the gap is below the requirement).
     * Why: a failed route has to be explainable to a student reading the log.
     */
    @Test
    public void aViolationSaysWhereAndWhat() {
        RouteSweep.Violation v = plain().check(route(Pose2.ofDegrees(20, 20, -90), -90, 60, 20, 60, 5));
        assertNotNull(v);
        assertTrue(v.distanceIn > 0.0);
        assertNotNull(v.pose);
        assertTrue(v.what.startsWith("wall"));
        assertTrue(v.gapIn < v.requiredIn);
        assertTrue(v.toString().contains(v.what));
    }

    // ---------------------------------------------------------------- the signed gap

    /**
     * Situation: the gap between a square at known poses and a box, worked out by hand.
     * Expected: an upright square 4 in left of the box has a gap of 4; touching has 0; overlapping by 3 in has -3;
     *     the same square turned 45 degrees and 4 in (centre to near edge) away has a gap that never exceeds the true
     *     distance.
     * Why: the whole check is built on this number, so its sign and size are pinned directly.
     */
    @Test
    public void theGapIsPositiveApartZeroTouchingAndNegativeOverlapping() {
        Rect box = new Rect(20, 40, 0, 20);
        assertEquals(4.0, RouteSweep.gapToRect(Footprint.of(Pose2.ofDegrees(20 - 4 - HALF, 10, 0), SQUARE), box), EPS);
        assertEquals(0.0, RouteSweep.gapToRect(Footprint.of(Pose2.ofDegrees(20 - HALF, 10, 0), SQUARE), box), EPS);
        assertEquals(-3.0, RouteSweep.gapToRect(Footprint.of(Pose2.ofDegrees(20 - HALF + 3, 10, 0), SQUARE), box), EPS);
        // Turned 45 degrees the square's corner (12.58 in from its centre) points at the box.
        double reach = HALF * Math.sqrt(2.0);
        double gap = RouteSweep.gapToRect(Footprint.of(Pose2.ofDegrees(20 - reach - 4, 10, 45), SQUARE), box);
        assertEquals(4.0, gap, 1e-9);
        assertTrue(gap <= 4.0 + EPS);
    }

    // ---------------------------------------------------------------- agreement with the slow referee

    /**
     * Situation: 400 random routes (random clear start, 1 to 3 waypoints, random final heading) on a 100 in field with
     *     two boxes in it, with no margins at all, so the check is about real contact.
     * Expected: (1) whenever the check says SAFE, the independent referee finds no overlap beyond the tolerance (the check
     *     never lets a clip through); (2) whenever the check says UNSAFE with a gap worse than -0.2 in, the referee also
     *     finds a real clip. The test also insists both outcomes occur many times, so it is not vacuous.
     * Why: this is the guarantee the planner relies on: "safe" means safe, and "unsafe" is not a false alarm.
     */
    @Test
    public void agreesWithTheSlowRefereeOnRandomRoutes() {
        List<Rect> boxes = Arrays.asList(new Rect(40, 60, 40, 60), new Rect(75, 80, 10, 30));
        RouteSweep s = sweep(BOX, 0.0, boxes, 0.0);
        Random rng = new Random(2024);
        int safe = 0;
        int unsafe = 0;
        int trials = 0;
        while (trials < 400) {
            Pose2 start = new Pose2(10 + rng.nextDouble() * 80, 10 + rng.nextDouble() * 80, (rng.nextDouble() - 0.5) * 6.0);
            // Only starts that are clear of everything (a start inside a wall is a different question).
            if (BruteForceFootprint.wallPoke(start, SQUARE, BOX) > 0.0) {
                continue;
            }
            boolean startClear = true;
            for (Rect b : boxes) {
                if (BruteForceFootprint.overlaps(start, SQUARE, b, 0.0)) {
                    startClear = false;
                }
            }
            if (!startClear) {
                continue;
            }
            trials++;
            List<double[]> wps = new ArrayList<double[]>();
            int n = 1 + rng.nextInt(3);
            for (int i = 0; i < n; i++) {
                wps.add(new double[]{10 + rng.nextDouble() * 80, 10 + rng.nextDouble() * 80});
            }
            HeadingProfile p = HeadingProfile.build(start, wps, (rng.nextDouble() - 0.5) * 6.0);
            RouteSweep.Violation v = s.check(p);
            String clip = BruteForceFootprint.firstClip(p, SQUARE, BOX, boxes, TOL);
            if (v == null) {
                safe++;
                assertNull("the check let a clip through: " + clip, clip);
            } else {
                unsafe++;
                if (v.gapIn < -0.2) {
                    assertNotNull("the check reported a clip the referee cannot see: " + v, BruteForceFootprint.firstClip(
                            p, SQUARE, BOX, boxes, TOL / 2.0));
                }
            }
        }
        assertTrue("safe routes " + safe, safe > 20);
        assertTrue("unsafe routes " + unsafe, unsafe > 20);
    }
}
