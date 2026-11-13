package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The robot is NOT centered on its position (its center of rotation): the front edge is 9.77 in from it, the back edge 8.03
 * in, each side 8.81 in (the numbers come from BioBuzzConstants). This class checks that every piece of geometry that decides "does the robot fit here?" uses those four
 * numbers: {@link Footprint}, {@link RouteSweep} and {@link ScoringPlanner} (its standing check and its wall limit).
 *
 * <p>How it works: no robot. Poses are built so the outline touches a wall (or sits a known gap off it) for every one of the four
 * facing directions at every wall and every corner, using {@link WallStarts}. The slow, independent {@link BruteForceFootprint}
 * referee measures how far the corners stick out; the code under test must agree with it. Each check has a negative twin that
 * moves the robot a small step closer and must fail, so a check that quietly used half the length or half the width would be
 * caught (it would be wrong by 0.87 in for the front and the back).
 *
 * <p>Words used: <b>facing</b> is the direction the intake points (0 = toward +x, 90 = toward +y); <b>gap</b> is the empty space
 * between the outline and the wall in inches (0 = touching); <b>clearance</b> is the gap the planner insists on.
 */
public class AsymmetricRobotTest {
    private static final double EPS = 1e-9;
    private static final RobotShape ROBOT = RobotShape.fromConstants();
    private static final double FRONT = ROBOT.frontIn();
    private static final double REAR = ROBOT.rearIn();
    private static final double SIDE = ROBOT.leftIn();
    private static final int[] FACINGS = {0, 90, 180, -90};

    private final FieldModel field = new FieldModel();
    private final ScoringPlanner.Config cfg = ScoringPlanner.Config.fromConstants(ShotTable.fromConstants());
    private final ScoringPlanner planner = new ScoringPlanner(field, cfg);

    // ---- the four distances ---------------------------------------------------------------------------------------

    /**
     * Situation: the robot touching the audience wall (y = 0) facing each of the four directions.
     * Expected: the center of rotation is 9.77 in from the wall facing the wall, 8.03 in facing away, 8.81 in sideways (either way).
     *     The rear wall (y = Field.SIZE_IN) gives the same numbers from the other side.
     */
    @Test
    public void theCenterIsADifferentDistanceFromTheWallForEachFacing() {
        double[][] expected = {{0, SIDE}, {90, REAR}, {180, SIDE}, {-90, FRONT}};
        for (double[] e : expected) {
            Pose2 audience = WallStarts.againstWall(field, ROBOT, 0, 60.0, e[0], 0.0);
            assertEquals("audience wall, facing " + e[0], e[1], audience.y, EPS);
            Pose2 rear = WallStarts.againstWall(field, ROBOT, 1, 60.0, e[0], 0.0);
            // facing +90 the FRONT is toward the rear wall; facing -90 the back is
            double expectedRear = e[0] == 90 ? FRONT : e[0] == -90 ? REAR : SIDE;
            assertEquals("rear wall, facing " + e[0], BioBuzzConstants.Field.SIZE_IN - expectedRear, rear.y, EPS);
            Pose2 red = WallStarts.againstWall(field, ROBOT, 2, 60.0, e[0], 0.0);
            double expectedRed = e[0] == 180 ? FRONT : e[0] == 0 ? REAR : SIDE;
            assertEquals("red wall, facing " + e[0], expectedRed, red.x, EPS);
            Pose2 blue = WallStarts.againstWall(field, ROBOT, 3, 60.0, e[0], 0.0);
            double expectedBlue = e[0] == 0 ? FRONT : e[0] == 180 ? REAR : SIDE;
            assertEquals("blue wall, facing " + e[0], BioBuzzConstants.Field.SIZE_IN - expectedBlue, blue.x, EPS);
        }
    }

    // ---- Footprint against the referee -------------------------------------------------------------------------

    /**
     * Situation: 12 headings (every 30 degrees) at every wall (three places each), every corner, touching and 0.3 in too close.
     * Expected: the referee says the touching outline pokes through nothing (0 in) and the 0.3 in version pokes 0.3 in, and
     *     {@link Footprint#signedOvershoot} says the same; the first facing the wall front-first is 1.74 in farther out than the
     *     back-first one.
     */
    @Test
    public void everyFacingAtEveryWallAndCornerTouchesExactlyAndPokesWhenNudgedIn() {
        Rect box = field.fieldRect();
        int checked = 0;
        for (Pose2 touching : WallStarts.all(field, ROBOT)) {
            // WallStarts.all lists poses at gap 0 and gap 1.5; only the touching ones have the property below
            double poke = BruteForceFootprint.wallPoke(touching, ROBOT, box);
            double nearest = Math.min(Math.min(box.maxX - Footprint.of(touching).bounds().maxX, Footprint.of(touching).bounds().minX - box.minX),
                    Math.min(box.maxY - Footprint.of(touching).bounds().maxY, Footprint.of(touching).bounds().minY - box.minY));
            if (nearest > 1e-6) {
                continue;                                   // this one is the 1.5 in gap variant
            }
            assertEquals("touching " + touching, 0.0, poke, 1e-9);
            assertEquals("signed overshoot of the touching outline " + touching, 0.0, Footprint.of(touching).signedOvershoot(box), 1e-9);
            checked++;
            // push it 0.3 in toward every wall it touches: the corners that touched now stick out by 0.3
            Rect b = Footprint.of(touching).bounds();
            double dx = Math.abs(b.minX - box.minX) < 1e-6 ? -0.3 : Math.abs(box.maxX - b.maxX) < 1e-6 ? 0.3 : 0.0;
            double dy = Math.abs(b.minY - box.minY) < 1e-6 ? -0.3 : Math.abs(box.maxY - b.maxY) < 1e-6 ? 0.3 : 0.0;
            Pose2 nudged = new Pose2(touching.x + dx, touching.y + dy, touching.heading);
            assertEquals("0.3 in in " + touching, 0.3, BruteForceFootprint.wallPoke(nudged, ROBOT, box), 1e-9);
            assertEquals(0.3, Footprint.of(nudged).signedOvershoot(box), 1e-9);
        }
        // 12 headings x (4 walls x 3 places + 4 corners) = 192 touching starts
        assertEquals(192, checked);
    }

    // ---- ScoringPlanner: the standing check and the wall limit ----------------------------------------------------

    /**
     * Situation: the planner's standing check at the wall clearance (0.25 in) for every wall, corner and facing, then 0.05 in closer.
     * Expected: free at the clearance, not free 0.05 in closer.
     * Why: a planner that used half the length for the wall limit would call the front-first pose 0.87 in too close to the wall
     *     free (it would let the front poke 0.87 in through) and the back-first one 0.87 in too far.
     */
    @Test
    public void thePlannerKeepsTheClearanceFromEveryWallForEveryFacing() {
        double clearance = cfg.wallClearanceIn;
        int checked = 0;
        for (int deg : FACINGS) {
            for (int wall = 0; wall < 4; wall++) {
                for (double along : new double[] {30.0, 72.0, 114.0}) {
                    Pose2 ok = WallStarts.againstWall(field, ROBOT, wall, along, deg, clearance + 1e-6);
                    assertTrue("free at the clearance: wall " + wall + " facing " + deg + " " + ok, planner.standingPoseFree(ok));
                    Pose2 tooClose = WallStarts.againstWall(field, ROBOT, wall, along, deg, clearance - 0.05);
                    assertFalse("0.05 in closer: wall " + wall + " facing " + deg + " " + tooClose, planner.standingPoseFree(tooClose));
                    checked++;
                }
            }
            for (int corner = 0; corner < 4; corner++) {
                Pose2 ok = WallStarts.inCorner(field, ROBOT, corner, deg, clearance + 1e-6);
                assertTrue("free in corner " + corner + " facing " + deg, planner.standingPoseFree(ok));
                assertFalse("0.05 in closer in corner " + corner + " facing " + deg,
                        planner.standingPoseFree(WallStarts.inCorner(field, ROBOT, corner, deg, clearance - 0.05)));
                checked++;
            }
        }
        assertEquals(4 * (12 + 4), checked);
    }

    /**
     * Situation: the planner is asked for the in-line audience-cell shot with a range band wide enough and a fine enough step that
     *     the wall is the only limit.
     * Expected: it stands with its FRONT edge on the wall clearance: the center of rotation 9.77 + 0.25 = 10.02 in from the wall,
     *     and the ball exit 5.38 in farther out (15.40 in from the wall), so (aim point 54.16 in from the wall) 38.76 in from the aim point.
     * Negative twin: told the robot is a centered rectangle (front half the length, 8.9 in) but still given the real ball exit, the same planner
     *     would stand 0.87 in closer to the wall (9.15 in), which puts the real front edge 0.87 in through it.
     */
    @Test
    public void theWallLimitOfTheShotUsesTheFrontExtentWhenTheRobotFacesTheWall() {
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        ScoringPlanner.Config c = ScoringPlanner.Config.fromConstants(ShotTable.fromConstants());
        c.rangeMinIn = 34.0;
        c.rangeMaxIn = 42.0;
        c.rangeStepIn = 0.01;
        c.coneHalfAngleDeg = 0.0;
        c.preferredRangeIn = 42.0;                      // farther than the wall allows, so the wall is what limits the range
        c.rangeCostPerIn = 100.0;                       // and being short of it costs far more than any driving, so the planner goes to the limit
        Pose2 start = new Pose2(cell.axisX, FRONT, -Math.PI / 2.0);
        ScoringPlanner.Solution s = new ScoringPlanner(field, c).solve(start, cell);
        assertTrue(s.toString(), s.ok());
        assertEquals("center of rotation 9.77 + 0.25 in from the wall", FRONT + c.wallClearanceIn, s.pose.y, 0.02);
        double exitBehind = -BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN;           // the measured ball exit, 5.38 in behind the center of rotation
        assertEquals("ball exit " + exitBehind + " in farther out", FRONT + c.wallClearanceIn + exitBehind, ShotGeometry.launchY(s.pose, c.launchForwardIn, c.launchLeftIn), 0.02);
        assertEquals(cell.originY - (FRONT + c.wallClearanceIn + exitBehind), s.rangeIn, 0.02);
        assertEquals(-90.0, s.pose.headingDegrees(), 1e-6);
        // the real front edge is exactly on the clearance
        assertEquals(c.wallClearanceIn, Footprint.of(s.pose).bounds().minY, 0.02);

        // the old assumption (a centered robot, front edge half the length from the center) with the real ball exit
        ScoringPlanner.Config centered = ScoringPlanner.Config.fromConstants(ShotTable.fromConstants());
        centered.rangeMinIn = c.rangeMinIn;
        centered.rangeMaxIn = c.rangeMaxIn;
        centered.rangeStepIn = c.rangeStepIn;
        centered.coneHalfAngleDeg = 0.0;
        centered.preferredRangeIn = c.preferredRangeIn;
        centered.rangeCostPerIn = c.rangeCostPerIn;
        centered.robot = RobotShape.centered(BioBuzzConstants.Robot.LENGTH_IN, BioBuzzConstants.Robot.WIDTH_IN);
        ScoringPlanner.Solution wrong = new ScoringPlanner(field, centered).solve(start, cell);
        assertTrue(wrong.ok());
        assertEquals("the centered planner stands 0.87 in too close to the wall", s.pose.y - (FRONT - BioBuzzConstants.Robot.LENGTH_IN / 2.0), wrong.pose.y, 0.02);
        double realFrontEdge = Footprint.of(wrong.pose).bounds().minY;
        assertTrue("with the real extents that pose would put the front edge " + (-realFrontEdge) + " in through the wall", realFrontEdge < -0.5);
    }

    // ---- RouteSweep: a leg along each wall, facing every way ---------------------------------------------------

    private RouteSweep sweepWith(double clearance) {
        return new RouteSweep(field.fieldRect(), clearance, field.hardObstacles(), cfg.keepOutMarginIn, ROBOT, RouteSweep.Settings.fromConstants());
    }

    private static HeadingProfile slide(Pose2 from, Pose2 to) {
        List<double[]> waypoints = new ArrayList<double[]>();
        waypoints.add(new double[] {to.x, to.y});
        return HeadingProfile.build(from, waypoints, from.heading);
    }

    /**
     * Situation: a 30 in slide along each wall, facing each of the four directions, kept 0.5 in off the wall (a legal run), then
     *     the same slide ending 0.5 in INSIDE the wall.
     * Expected: the sweep finds the first one safe and the second one a violation of exactly that wall, and the independent referee
     *     agrees (it sees no clip in the first, a clip of more than the tolerance in the second).
     */
    @Test
    public void aSlideAlongAnyWallFacingAnyWayIsSafeWhenClearAndCaughtWhenItGoesIn() {
        // where along each wall a 30 in slide stays clear of the FLOWER plates and the HIVE: {wall, from, to}
        double[][] runs = {{0, 30, 60}, {1, 70, 100}, {2, 70, 100}, {3, 30, 60}};
        RouteSweep sweep = sweepWith(cfg.wallClearanceIn);
        double tol = BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN + 1e-6;
        int checked = 0;
        for (double[] run : runs) {
            for (int deg : FACINGS) {
                int wall = (int) run[0];
                Pose2 a = WallStarts.againstWall(field, ROBOT, wall, run[1], deg, 0.5);
                Pose2 b = WallStarts.againstWall(field, ROBOT, wall, run[2], deg, 0.5);
                HeadingProfile clear = slide(a, b);
                assertNull("wall " + wall + " facing " + deg + ": " + sweep.check(clear), sweep.check(clear));
                assertNull(BruteForceFootprint.firstClip(clear, ROBOT, field.fieldRect(), field.hardObstacles(), tol));

                Pose2 c = WallStarts.againstWall(field, ROBOT, wall, run[2], deg, -0.5);
                HeadingProfile into = slide(a, c);
                RouteSweep.Violation v = sweep.check(into);
                assertNotNull("wall " + wall + " facing " + deg + " must be caught", v);
                assertNotNull("the referee sees the clip too", BruteForceFootprint.firstClip(into, ROBOT, field.fieldRect(), field.hardObstacles(), tol));
                checked++;
            }
        }
        assertEquals(16, checked);
    }

    /**
     * Situation: the robot spinning in place at a distance from the audience wall.
     * Expected: it may turn freely only when its center of rotation is at least the circumscribed radius (13.16 in, a FRONT corner:
     *     hypot(9.77, 8.81)) from the wall plus the clearance; one inch closer it cannot. The referee finds the real spin clips by more than
     *     the tolerance there and does not at that distance.
     * Why: spinning sweeps the farthest corner around a circle of 13.16 in, not the 12.52 in of a centered rectangle.
     */
    @Test
    public void spinningInPlaceNeedsTheFullCircumscribedRadiusFromAWall() {
        RouteSweep sweep = sweepWith(cfg.wallClearanceIn);
        double r = sweep.reachIn();
        assertEquals(ROBOT.reachIn(), r, EPS);
        assertEquals(Math.hypot(Math.max(FRONT, REAR), Math.max(ROBOT.leftIn(), ROBOT.rightIn())), r, 0.001);      // the farthest corner: a front one
        double x = 60.0;
        assertTrue(sweep.anyHeadingIsSafeAt(x, r + cfg.wallClearanceIn + 0.01));
        assertFalse(sweep.anyHeadingIsSafeAt(x, r + cfg.wallClearanceIn - 0.01));
        // the referee: spin the real outline in 1 degree steps at those two distances
        double tol = BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN;
        assertTrue(spinPoke(x, r + 0.01) <= 1e-9);
        assertTrue("a spin one inch too close pokes " + spinPoke(x, r - 1.0), spinPoke(x, r - 1.0) > 0.9 - tol);
        // with the centered numbers (half the length, half the width) the radius would be 12.52 in and the spin 0.6 in short of the real room
        assertTrue(Math.hypot(BioBuzzConstants.Robot.LENGTH_IN / 2.0, BioBuzzConstants.Robot.WIDTH_IN / 2.0) < r - 0.5);
    }

    private double spinPoke(double x, double y) {
        double worst = 0.0;
        for (int deg = 0; deg < 360; deg++) {
            worst = Math.max(worst, BruteForceFootprint.wallPoke(Pose2.ofDegrees(x, y, deg), ROBOT, field.fieldRect()));
        }
        return worst;
    }

    // ---- the TeleOp default start ---------------------------------------------------------------------------------------

    @Test
    public void theTeleOpDefaultStartStandsFreeAndTouchesTheWall() {
        Pose2 start = Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN, BioBuzzConstants.Controls.DEFAULT_START_Y_IN,
                BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG);
        assertEquals("the front edge is on the audience wall", 0.0, BruteForceFootprint.wallPoke(start, ROBOT, field.fieldRect()), EPS);
        assertEquals(0.0, Footprint.of(start).bounds().minY, EPS);
    }
}
