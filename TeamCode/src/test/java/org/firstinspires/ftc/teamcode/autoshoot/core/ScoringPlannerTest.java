package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.junit.Test;

/**
 * Checks where the robot decides to stand when you hold the shoot button: {@link ShotGeometry} (distances and angles
 * between the robot, its ball exit and the aim point), {@link PathPlanner} (a route around obstacles) and
 * {@link ScoringPlanner} (picks the cheapest standing pose that is allowed).
 *
 * <p>How it works: no robot. Each test builds the real field map and shot table, asks the planner for a pose from a
 * chosen starting pose, and checks the answer two ways: against numbers worked out by hand, and against rules that
 * every answer must obey (inside the shooting cone, clear of the HIVE frame, shooter pointed at the aim point).
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*ScoringPlannerTest"} to run only
 * this class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Aim point:</b> the center of an UP cell's opening, the spot a ball should pass through.</li>
 *   <li><b>Ball exit (launch point):</b> where the ball leaves the shooter. The shooter fires out the BACK, so the
 *       exit is behind the robot's center (forward is negative).</li>
 *   <li><b>Range:</b> the horizontal distance in inches from the ball exit to the aim point.</li>
 *   <li><b>Cone, azimuth:</b> the robot may stand anywhere inside a cone around the opening's center line. Azimuth
 *       is how many degrees off that center line a pose is.</li>
 *   <li><b>Footprint:</b> the floor space the robot covers. The planner uses the rectangle that just contains it
 *       (its 'axis-aligned box').</li>
 *   <li><b>Keep-out:</b> an area the robot must stay out of (the HIVE frame, the FLOWERS, the walls).</li>
 *   <li><b>Waypoint:</b> one point on a route. The robot drives through them in order.</li>
 *   <li><b>Feasible:</b> a standing pose that obeys every rule and has a clear route to it.</li>
 *   <li><b>Tolerance / EPS (epsilon):</b> how far two decimal numbers may differ and still count as equal, to allow
 *       for rounding. Here one millionth.</li>
 *   <li><b>Fixture:</b> the shared setup for the tests (the fields below). JUnit builds a fresh copy for every
 *       test.</li>
 * </ul>
 */
public class ScoringPlannerTest {
    /** Allowed rounding error when comparing two decimal numbers: one millionth. */
    private static final double EPS = 1e-6;

    /**
     * The shared fixture (these four fields): the real field map, shot table, planner settings and planner, all built
     * from BioBuzzConstants.
     */
    private final FieldModel field = new FieldModel();
    private final ShotTable table = ShotTable.fromConstants();
    private final ScoringPlanner.Config cfg = ScoringPlanner.Config.fromConstants(table);
    private final ScoringPlanner planner = new ScoringPlanner(field, cfg);

    /**
     * The C1 start the team's autos use, written the right way for the real robot: facing the audience wall (-90 degrees)
     * with its FRONT edge on the wall, so the robot's position (its center of rotation) is the front extent, 9.77 in, from it.
     */
    private static Pose2 c1Start() {
        return Pose2.ofDegrees(58.0, Footprint.frontExtentIn(), -90);
    }

    /** The B1 start, the same way: front edge on the audience wall. */
    private static Pose2 b1Start() {
        return Pose2.ofDegrees(35.0, Footprint.frontExtentIn(), -90);
    }

    // ---------------------------------------------------------------- ShotGeometry

    /**
     * Situation: the robot at the C1 autonomous start (58, 9.77) facing -90 degrees (front edge on the audience wall), with
     *     the RED audience cell as the target.
     * Expected: the range from the ball exit to the aim point is about 36.36 in (the ball exit is 17.80 in from the wall,
     *     so 54.156 - 17.80 = 36.356 in straight up, plus 0.076 in sideways: x = 58 against the HIVE axis 57.924), and the
     *     shooter is within 3.5 degrees of pointing at it.
     * Why: the shooter is on the back, so aiming means heading = bearing + half a turn. Getting that wrong would aim
     *     every shot 180 degrees away. Tolerance: 0.05 in on the range; 3.5 degrees is a loose limit (C1 is almost on
     *     the HIVE axis, so the real error is a fraction of a degree).
     */
    @Test
    public void shooterPointsAtTheAimPointWhenHeadingIsBearingPlusHalfATurn() {
        // Robot at C1 facing -90: the back faces +y, straight at the red audience cell's aim point.
        Pose2 c1 = c1Start();
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        double range = ShotGeometry.rangeTo(c1, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        double exitY = c1.y - cfg.launchForwardIn;             // the ball exit, a whole robot length from the wall
        assertEquals(Math.hypot(c1.x - cell.originX, cell.originY - exitY), range, EPS);
        assertEquals(36.36, range, 0.05);                      // 0.076 in off the axis, ball exit 17.80 in from the wall
        double err = Math.toDegrees(ShotGeometry.aimError(c1, cfg.launchForwardIn, cfg.launchLeftIn,
                cell.originX, cell.originY));
        assertTrue("C1 is nearly in line: " + err, Math.abs(err) < 3.5);
    }

    /**
     * Situation: ask for the pose whose ball exit is at (50, 20) with the shooter pointing at the aim point
     *     (57.924, 54.156), using a launch point 8.9 in behind and 1.5 in to the left of the robot center.
     * Expected: working out the ball exit of that pose gives back (50, 20), and the aim error is zero.
     * Why: the planner works backward from 'where the ball exit should be' to 'where the robot center goes'. This
     *     proves the forward and backward calculations agree. 1e-6 is rounding only.
     */
    @Test
    public void poseForLaunchPointRoundTripsTheExitAndAimDirection() {
        Pose2 p = ShotGeometry.poseForLaunchPoint(50, 20, 57.924, 54.156, -8.9, 1.5);
        assertEquals(50, ShotGeometry.launchX(p, -8.9, 1.5), EPS);
        assertEquals(20, ShotGeometry.launchY(p, -8.9, 1.5), EPS);
        assertEquals(0.0, ShotGeometry.aimError(p, -8.9, 1.5, 57.924, 54.156), EPS);
    }

    // ---------------------------------------------------------------- PathPlanner

    /**
     * Situation: a 100 x 100 in field with one 20 x 20 in box in the middle; start (10, 10), goal (90, 20), a line that
     *     misses the box.
     * Expected: the route is just the goal point.
     * Why: the planner must not invent detours. A straight line is shortest.
     */
    @Test
    public void straightRouteWhenNothingIsInTheWay() {
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Collections.singletonList(new Rect(40, 60, 40, 60)));
        List<double[]> r = pp.plan(10, 10, 90, 20);
        assertNotNull(r);
        assertEquals(1, r.size());
        assertEquals(90, r.get(0)[0], EPS);
        assertEquals(20, r.get(0)[1], EPS);
    }

    /**
     * Situation: the same field; start (10, 50) and goal (90, 50) with the box directly in between.
     * Expected: the route has at least 2 points (it bends around a corner of the box), its length is between 80 in (the
     *     blocked straight line) and 95 in, and no leg of it crosses the box.
     * Why: the robot must go around the HIVE frame, not through it, and without a silly long detour.
     */
    @Test
    public void routeBendsAroundAnObstacleThroughItsCorners() {
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Collections.singletonList(new Rect(40, 60, 40, 60)));
        List<double[]> r = pp.plan(10, 50, 90, 50);
        assertNotNull(r);
        assertTrue("should detour", r.size() >= 2);
        double len = PathPlanner.length(10, 50, r);
        assertTrue(len > 80.0);
        assertTrue("detour should be short, was " + len, len < 95.0);
        // No leg crosses the obstacle.
        double x = 10, y = 50;
        Rect o = new Rect(40, 60, 40, 60);
        for (double[] w : r) {
            assertFalse(o.segmentCrossesInterior(x, y, w[0], w[1]));
            x = w[0];
            y = w[1];
        }
    }

    /**
     * Situation: a goal inside the box, and a goal outside the field.
     * Expected: plan() returns null (no route) for both.
     * Why: the planner must say 'impossible' instead of driving the robot into a keep-out or off the field.
     */
    @Test
    public void noRouteIntoAnObstacleOrOutOfBounds() {
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Collections.singletonList(new Rect(40, 60, 40, 60)));
        assertNull(pp.plan(10, 10, 50, 50));      // goal inside
        assertNull(pp.plan(10, 10, 150, 50));     // goal outside
    }

    /**
     * Situation: the START point (42, 50) is inside the box (as if the robot had been nudged into a keep-out margin).
     * Expected: the first waypoint is just outside the box, and the route goes on from there to the goal.
     * Why: the robot cannot be assumed to start in free space. It has to step out first.
     */
    @Test
    public void startInsideAnObstacleStepsOutFirst() {
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Collections.singletonList(new Rect(40, 60, 40, 60)));
        List<double[]> r = pp.plan(42, 50, 90, 50);
        assertNotNull(r);
        assertTrue(r.size() >= 2);
        double[] first = r.get(0);
        assertFalse(new Rect(40, 60, 40, 60).containsStrictly(first[0], first[1]));
    }

    /**
     * Situation: a barrier that spans the whole field width between the start (50, 10) and the goal (50, 90).
     * Expected: plan() returns null.
     * Why: no route exists, so the planner must not return a path through the barrier.
     */
    @Test
    public void wallsOffGoalIsUnreachable() {
        // A full-width wall splits the field.
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Arrays.asList(new Rect(-5, 105, 45, 55)));
        assertNull(pp.plan(50, 10, 50, 90));
    }

    // ---------------------------------------------------------------- ScoringPlanner

    /**
     * The rules every planner answer must obey, whatever the start: it is a real solution; the range is inside the
     * allowed band; the pose is inside the shooting cone; the shooter points exactly at the aim point; the robot's
     * footprint is clear of the HIVE frame, the FLOWERS and the walls; the route ends at the pose; and the ball exit is
     * on the side the opening faces. Used by several tests below.
     */
    private void assertGoodSolution(ScoringPlanner.Solution s, CellSpec cell) {
        assertTrue("expected a solution: " + s, s.ok());
        // Range inside the allowed band.
        // (1e-6 in of slack so rounding at the edges of the band is not a failure.)
        assertTrue(s.rangeIn >= cfg.rangeMinIn - 1e-6 && s.rangeIn <= cfg.rangeMaxIn + 1e-6);
        // Inside the cone.
        assertTrue(Math.abs(s.azimuthDeg) <= cfg.coneHalfAngleDeg + 1e-6);
        // Shooter really points at the aim point.
        double err = ShotGeometry.aimError(s.pose, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        // (1e-6 radians: rounding only. The planner builds the pose so the aim error is exactly zero.)
        assertEquals(0.0, err, 1e-6);
        // Footprint is free of the HIVE frame, the FLOWERS and the walls.
        assertTrue(planner.standingPoseFree(s.pose));
        Rect box = ShotGeometry.footprint(s.pose, cfg.robot);
        for (Rect o : field.hardObstacles()) {
            assertFalse("overlaps an obstacle: " + o, box.overlaps(o));
        }
        // The route ends at the pose.
        double[] last = s.waypoints.get(s.waypoints.size() - 1);
        assertEquals(s.pose.x, last[0], EPS);
        assertEquals(s.pose.y, last[1], EPS);
        // The ball exit is on the side the opening faces.
        double ly = ShotGeometry.launchY(s.pose, cfg.launchForwardIn, cfg.launchLeftIn);
        if (cell.side == Side.AUDIENCE) {
            assertTrue(ly < cell.originY);
        } else {
            assertTrue(ly > cell.originY);
        }
    }

    /**
     * Situation: the robot starts at the C1 autonomous start (58, 9.2, -90), almost already in place for the RED
     *     audience cell.
     * Expected: a good solution, under 5 in of travel, ending at a heading within 4 degrees of -90.
     * Why: if the robot is nearly in place, auto-shoot must not make big, pointless moves. The 4 degrees allows the
     *     small turn needed to point exactly at the aim point.
     */
    @Test
    public void fromTheC1StartTheRedAudienceCellNeedsAlmostNoMovement() {
        Pose2 c1 = c1Start();
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        ScoringPlanner.Solution s = planner.solve(c1, cell);
        assertGoodSolution(s, cell);
        assertTrue("travel " + s.travelIn, s.travelIn < 5.0);
        assertEquals(-90.0, s.pose.headingDegrees(), 4.0);
    }

    /**
     * Situation: the robot starts at the B1 autonomous start (35, 9.77, -90), about 23 in from the HIVE axis (x =
     *     57.92), toward the red wall.
     * Expected: a good solution that ends further toward the HIVE axis (larger x) with less than 25 in of travel.
     * Why: it should step sideways into the shooting cone, but it need not go all the way to the axis because the cone
     *     allows some angle off the center line.
     */
    @Test
    public void fromTheB1StartTheRobotMovesTowardTheCellAxisAndStaysInTheCone() {
        Pose2 b1 = b1Start();
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        ScoringPlanner.Solution s = planner.solve(b1, cell);
        assertGoodSolution(s, cell);
        assertTrue(s.pose.x > b1.x);                            // moved toward the HIVE axis (x = 57.92)
        assertTrue("travel " + s.travelIn, s.travelIn < 25.0);
    }

    /**
     * Situation: a RED start (40, 15, -100) planning for the RED audience cell, and the half-turned BLUE start planning
     *     for the BLUE rear cell.
     * Expected: the blue answer is exactly the half-turn of the red answer: same range, and a pose rotated half a turn
     *     about the field center.
     * Why: the numbers are written once, for RED. BLUE must behave identically on the opposite side.
     */
    @Test
    public void theBlueSolutionIsTheHalfTurnOfTheRedSolution() {
        Pose2 red = Pose2.ofDegrees(40.0, 15.0, -100);
        ScoringPlanner.Solution sr = planner.solve(red, field.cell(Alliance.RED, Side.AUDIENCE));
        Pose2 blue = Alliance.BLUE.fromRed(red, field.size());
        ScoringPlanner.Solution sb = planner.solve(blue, field.cell(Alliance.BLUE, Side.REAR));
        assertTrue(sr.ok() && sb.ok());
        Pose2 expected = Alliance.BLUE.fromRed(sr.pose, field.size());
        assertEquals(expected.x, sb.pose.x, 1e-6);
        assertEquals(expected.y, sb.pose.y, 1e-6);
        assertEquals(0.0, Math.toDegrees(Angles.distance(expected.heading, sb.pose.heading)), 1e-6);
        assertEquals(sr.rangeIn, sb.rangeIn, 1e-6);
    }

    /**
     * Situation: for each alliance and each of its two cells (4 cases), start from that alliance's version of the C1
     *     start pose.
     * Expected: every one of the four gets a good solution.
     * Why: this covers the far-side cells (which need a route around the HIVE) and the blue alliance's rotated geometry
     *     in one sweep.
     */
    @Test
    public void everyCellOfEveryAlliancePlansFromItsOwnStartingCorner() {
        for (Alliance a : Alliance.values()) {
            for (Side side : Side.values()) {
                CellSpec cell = field.cell(a, side);
                Pose2 start = a.fromRed(c1Start(), field.size());
                ScoringPlanner.Solution s = planner.solve(start, cell);
                assertGoodSolution(s, cell);
            }
        }
    }

    /**
     * Situation: from the C1 start on the audience side to the RED REAR cell, on the other side of the HIVE.
     * Expected: a route of at least 2 points around the HIVE frame; the final pose is beyond the frame's far edge (y
     *     above 91.475); it faces about +90 degrees (within 30); the travel is between 100 and 175 in; and no leg
     *     crosses the HIVE frame grown by half the robot's diagonal.
     * Why: the far cell is reached by going around the HIVE. The grown box is how far the robot can reach while it
     *     turns on the way. The distance band is a loose sanity check: the straight-line distance is already well over
     *     100 in, and more than 175 in would be a silly detour.
     */
    @Test
    public void reachingTheFarSideGoesAroundTheHiveFrameAndStaysClear() {
        Pose2 c1 = c1Start();
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        ScoringPlanner.Solution s = planner.solve(c1, rear);
        assertGoodSolution(s, rear);
        assertTrue("needs bends: " + s.waypoints.size(), s.waypoints.size() >= 2);
        assertTrue(s.pose.y > field.hiveFrame().maxY);
        assertEquals(90.0, s.pose.headingDegrees(), 30.0);
        assertTrue("travel " + s.travelIn, s.travelIn > 100.0 && s.travelIn < 175.0);

        // No leg crosses the HIVE frame grown by the robot size.
        double halfDiag = cfg.robot.reachIn();
        Rect keepOut = field.hiveFrame().inflate(halfDiag);
        double x = c1.x, y = c1.y;
        for (double[] w : s.waypoints) {
            assertFalse(keepOut.segmentCrossesInterior(x, y, w[0], w[1]));
            x = w[0];
            y = w[1];
        }
    }

    /**
     * Situation: a test-only planner setting that insists on shooting from 45 to 60 in away, from C1 at the audience
     *     wall.
     * Expected: status NO_FEASIBLE_POSE, and ok() is false.
     * Why: the planner must say 'impossible' rather than return a pose that breaks the wall or cone rules. The
     *     controller then stops with a fault instead of driving nowhere.
     */
    @Test
    public void aRangeBandBeyondWhatTheWallAllowsHasNoFeasiblePose() {
        // The audience wall limits the shooter to about 37.7 in from the cell aim point; a 45-60 in band cannot be reached.
        ScoringPlanner.Config far = ScoringPlanner.Config.fromConstants(table);
        far.rangeMinIn = 45.0;
        far.rangeMaxIn = 60.0;
        ScoringPlanner p = new ScoringPlanner(field, far);
        ScoringPlanner.Solution s = p.solve(c1Start(), field.cell(Alliance.RED, Side.AUDIENCE));
        assertEquals(ScoringPlanner.Status.NO_FEASIBLE_POSE, s.status);
        assertFalse(s.ok());
    }

    /**
     * Situation: ranges from 36 to 60 in are allowed, but only in line with the opening (cone of 0 degrees), starting
     *     from C1.
     * Expected: a pose is found, but its range is under 38 in.
     * Why: this shows the limit comes from the geometry (the wall), not just from the shot table. The 38 in bound is
     *     the 37.7 in figure rounded up; the planner searches ranges in 1 in steps.
     */
    @Test
    public void theWallLimitOnRangeMatchesTheVerifiedGeometry() {
        // With the ball exit at the rear face and the robot against the audience wall the exit is 17.8 in from
        // the wall, so the farthest in-line range is 54.156 - 17.8 = 36.4 in with the CAD aim point (it was 37.7 in with the old
        // 144 in field, where the aim point was 55.52 in out; see MeasuredAnchorShotTest).
        ScoringPlanner.Config wide = ScoringPlanner.Config.fromConstants(table);
        wide.rangeMinIn = 36.0;
        wide.rangeMaxIn = 60.0;
        wide.coneHalfAngleDeg = 0.0;
        ScoringPlanner p = new ScoringPlanner(field, wide);
        ScoringPlanner.Solution s = p.solve(c1Start(), field.cell(Alliance.RED, Side.AUDIENCE));
        assertTrue(s.ok());
        assertTrue("range " + s.rangeIn, s.rangeIn < 38.0);
    }

    /**
     * The team's one measured good shot: robot front against the audience wall, in line with the HIVE. The shot table is
     * anchored at its measured range, 37.7 in (see MeasuredAnchorShotTest for how the CAD field numbers make the MODEL
     * say 36.36 in for the same place, 1.34 in short of the measurement, and what the checklist says about it).
     *
     * Situation: the robot with its front against the audience wall and exactly in line with the RED audience cell.
     * Expected: the model's range is 36.356 in (the CAD aim point 54.156 in from the wall minus the 17.80 in of the ball
     *     exit) and the shot table still gives 1200 tps at its anchor row, 37.7 in; the shooter is exactly in line (0
     *     degrees off the opening's center line); from there the planner moves under 2.5 in and picks a range within 1.1
     *     in of the wall one.
     * Why: the planner tries ranges in 1 in steps and keeps the robot a small WALL_CLEARANCE_IN off the wall, so it
     *     lands a little short of the wall range instead of exactly on it. That is why the range tolerance is 1.1 in.
     */
    @Test
    public void theRobotFrontAgainstTheWallInLineIsTheMeasuredAnchorShot() {
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 wallShot = new Pose2(cell.axisX, cfg.robot.frontIn(), -Math.PI / 2.0);
        double range = ShotGeometry.rangeTo(wallShot, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        assertEquals(36.356, range, 0.05);
        assertEquals(1200.0, table.velocityAt(37.7), EPS);
        assertEquals(0.0, Math.toDegrees(ShotGeometry.azimuthOffAxis(wallShot, cfg.launchForwardIn, cfg.launchLeftIn, cell)), 1e-6);
        // Starting there, the planner has nothing to do but a few inches of settling.
        ScoringPlanner.Solution s = planner.solve(wallShot, cell);
        assertTrue(s.ok());
        assertTrue("travel " + s.travelIn, s.travelIn < 2.5);   // a robot touching the wall is nudged off it by the wall clearance
        assertEquals(range, s.rangeIn, 1.1);
    }

    /**
     * Situation: the axis-aligned box (the rectangle lined up with the field's x and y axes) that just contains the real
     *     robot, for a robot standing at (60, 50) and turned different amounts. The robot is NOT centered on its position
     *     (its center of rotation): its edges are 9.77 in ahead, 8.03 in behind and 8.90 in to each side.
     * Expected: facing -90 degrees (front toward the audience wall, -y) the box reaches 9.77 in below the position and
     *     8.03 in above it; facing +90 it is the other way round; facing 0 it reaches 9.77 in toward +x. At 30 degrees the
     *     box equals the extremes of the four corners worked out by hand, and it is wider than the upright 17.8 in.
     * Why: the planner checks that this box is clear of walls and the HIVE frame. A box centered on the position would
     *     let the robot's front come 0.87 in closer to a wall than it really can, and keep its back 0.87 in away from a
     *     wall it is facing away from.
     */
    @Test
    public void footprintBoxIsNotCenteredOnThePositionAndGrowsWhenTheRobotIsTurned() {
        RobotShape robot = cfg.robot;
        Rect down = ShotGeometry.footprint(new Pose2(60, 50, -Math.PI / 2.0), robot);
        assertEquals(50 - 9.77, down.minY, EPS);
        assertEquals(50 + 8.03, down.maxY, EPS);
        assertEquals(60 - 8.90, down.minX, EPS);
        assertEquals(60 + 8.90, down.maxX, EPS);
        Rect up = ShotGeometry.footprint(new Pose2(60, 50, Math.PI / 2.0), robot);
        assertEquals(50 - 8.03, up.minY, EPS);
        assertEquals(50 + 9.77, up.maxY, EPS);
        Rect ahead = ShotGeometry.footprint(new Pose2(60, 50, 0.0), robot);
        assertEquals(60 - 8.03, ahead.minX, EPS);
        assertEquals(60 + 9.77, ahead.maxX, EPS);
        // turned 30 degrees: the four corners, worked out by hand
        double h = Math.toRadians(30);
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        for (double[] c : new double[][] {{9.77, 8.9}, {-8.03, 8.9}, {-8.03, -8.9}, {9.77, -8.9}}) {
            double x = 60 + c[0] * Math.cos(h) - c[1] * Math.sin(h);
            double y = 50 + c[0] * Math.sin(h) + c[1] * Math.cos(h);
            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
        }
        Rect turned = ShotGeometry.footprint(new Pose2(60, 50, h), robot);
        assertEquals(minX, turned.minX, EPS);
        assertEquals(maxX, turned.maxX, EPS);
        assertEquals(minY, turned.minY, EPS);
        assertEquals(maxY, turned.maxY, EPS);
        assertTrue("a turned robot needs a bigger box", turned.maxX - turned.minX > 17.8 + 1.0);
    }
}
