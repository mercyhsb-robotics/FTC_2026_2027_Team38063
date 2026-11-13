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

public class ScoringPlannerTest {
    private static final double EPS = 1e-6;

    private final FieldModel field = new FieldModel();
    private final ShotTable table = ShotTable.fromConstants();
    private final ScoringPlanner.Config cfg = ScoringPlanner.Config.fromConstants(table);
    private final ScoringPlanner planner = new ScoringPlanner(field, cfg);

    private double lengthIn() {
        return cfg.robotLengthIn;
    }

    // ---------------------------------------------------------------- ShotGeometry

    @Test
    public void shooterPointsAtTheAimPointWhenHeadingIsBearingPlusHalfATurn() {
        // Robot at C1 facing -90: the back faces +y, straight at the red audience cell's aim point.
        Pose2 c1 = Pose2.ofDegrees(58.0, 9.2, -90);
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        double range = ShotGeometry.rangeTo(c1, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        assertEquals(37.4, range, 0.05);                       // matches the verified "C1 start D = 37.4 in"
        double err = Math.toDegrees(ShotGeometry.aimError(c1, cfg.launchForwardIn, cfg.launchLeftIn,
                cell.originX, cell.originY));
        assertTrue("C1 is nearly in line: " + err, Math.abs(err) < 3.5);
    }

    @Test
    public void poseForLaunchPointRoundTripsTheExitAndAimDirection() {
        Pose2 p = ShotGeometry.poseForLaunchPoint(50, 20, 59.25, 55.52, -8.9, 1.5);
        assertEquals(50, ShotGeometry.launchX(p, -8.9, 1.5), EPS);
        assertEquals(20, ShotGeometry.launchY(p, -8.9, 1.5), EPS);
        assertEquals(0.0, ShotGeometry.aimError(p, -8.9, 1.5, 59.25, 55.52), EPS);
    }

    // ---------------------------------------------------------------- PathPlanner

    @Test
    public void straightRouteWhenNothingIsInTheWay() {
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Collections.singletonList(new Rect(40, 60, 40, 60)));
        List<double[]> r = pp.plan(10, 10, 90, 20);
        assertNotNull(r);
        assertEquals(1, r.size());
        assertEquals(90, r.get(0)[0], EPS);
        assertEquals(20, r.get(0)[1], EPS);
    }

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

    @Test
    public void noRouteIntoAnObstacleOrOutOfBounds() {
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Collections.singletonList(new Rect(40, 60, 40, 60)));
        assertNull(pp.plan(10, 10, 50, 50));      // goal inside
        assertNull(pp.plan(10, 10, 150, 50));     // goal outside
    }

    @Test
    public void startInsideAnObstacleStepsOutFirst() {
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Collections.singletonList(new Rect(40, 60, 40, 60)));
        List<double[]> r = pp.plan(42, 50, 90, 50);
        assertNotNull(r);
        assertTrue(r.size() >= 2);
        double[] first = r.get(0);
        assertFalse(new Rect(40, 60, 40, 60).containsStrictly(first[0], first[1]));
    }

    @Test
    public void wallsOffGoalIsUnreachable() {
        // A full-width wall splits the field.
        PathPlanner pp = new PathPlanner(new Rect(0, 100, 0, 100), Arrays.asList(new Rect(-5, 105, 45, 55)));
        assertNull(pp.plan(50, 10, 50, 90));
    }

    // ---------------------------------------------------------------- ScoringPlanner

    private void assertGoodSolution(ScoringPlanner.Solution s, CellSpec cell) {
        assertTrue("expected a solution: " + s, s.ok());
        // Range inside the allowed band.
        assertTrue(s.rangeIn >= cfg.rangeMinIn - 1e-6 && s.rangeIn <= cfg.rangeMaxIn + 1e-6);
        // Inside the cone.
        assertTrue(Math.abs(s.azimuthDeg) <= cfg.coneHalfAngleDeg + 1e-6);
        // Shooter really points at the aim point.
        double err = ShotGeometry.aimError(s.pose, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        assertEquals(0.0, err, 1e-6);
        // Footprint is free of the HIVE frame, the FLOWERS and the walls.
        assertTrue(planner.standingPoseFree(s.pose));
        Rect box = ShotGeometry.footprint(s.pose, cfg.robotLengthIn, cfg.robotWidthIn);
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

    @Test
    public void fromTheC1StartTheRedAudienceCellNeedsAlmostNoMovement() {
        Pose2 c1 = Pose2.ofDegrees(58.0, 9.2, -90);
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        ScoringPlanner.Solution s = planner.solve(c1, cell);
        assertGoodSolution(s, cell);
        assertTrue("travel " + s.travelIn, s.travelIn < 5.0);
        assertEquals(-90.0, s.pose.headingDegrees(), 4.0);
    }

    @Test
    public void fromTheB1StartTheRobotMovesTowardTheCellAxisAndStaysInTheCone() {
        Pose2 b1 = Pose2.ofDegrees(35.0, 9.2, -90);
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        ScoringPlanner.Solution s = planner.solve(b1, cell);
        assertGoodSolution(s, cell);
        assertTrue(s.pose.x > b1.x);                            // moved toward the HIVE axis (x = 59.25)
        assertTrue("travel " + s.travelIn, s.travelIn < 25.0);
    }

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

    @Test
    public void everyCellOfEveryAlliancePlansFromItsOwnStartingCorner() {
        for (Alliance a : Alliance.values()) {
            for (Side side : Side.values()) {
                CellSpec cell = field.cell(a, side);
                Pose2 start = a.fromRed(Pose2.ofDegrees(58.0, 9.2, -90), field.size());
                ScoringPlanner.Solution s = planner.solve(start, cell);
                assertGoodSolution(s, cell);
            }
        }
    }

    @Test
    public void reachingTheFarSideGoesAroundTheHiveFrameAndStaysClear() {
        Pose2 c1 = Pose2.ofDegrees(58.0, 9.2, -90);
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        ScoringPlanner.Solution s = planner.solve(c1, rear);
        assertGoodSolution(s, rear);
        assertTrue("needs bends: " + s.waypoints.size(), s.waypoints.size() >= 2);
        assertTrue(s.pose.y > field.hiveFrame().maxY);
        assertEquals(90.0, s.pose.headingDegrees(), 30.0);
        assertTrue("travel " + s.travelIn, s.travelIn > 100.0 && s.travelIn < 175.0);

        // No leg crosses the HIVE frame grown by the robot size.
        double halfDiag = 0.5 * Math.hypot(cfg.robotLengthIn, cfg.robotWidthIn);
        Rect keepOut = field.hiveFrame().inflate(halfDiag);
        double x = c1.x, y = c1.y;
        for (double[] w : s.waypoints) {
            assertFalse(keepOut.segmentCrossesInterior(x, y, w[0], w[1]));
            x = w[0];
            y = w[1];
        }
    }

    @Test
    public void aRangeBandBeyondWhatTheWallAllowsHasNoFeasiblePose() {
        // The audience wall limits the shooter to about 37.7 in from the cell aim point; a 45-60 in band cannot be reached.
        ScoringPlanner.Config far = ScoringPlanner.Config.fromConstants(table);
        far.rangeMinIn = 45.0;
        far.rangeMaxIn = 60.0;
        ScoringPlanner p = new ScoringPlanner(field, far);
        ScoringPlanner.Solution s = p.solve(Pose2.ofDegrees(58.0, 9.2, -90), field.cell(Alliance.RED, Side.AUDIENCE));
        assertEquals(ScoringPlanner.Status.NO_FEASIBLE_POSE, s.status);
        assertFalse(s.ok());
    }

    @Test
    public void theWallLimitOnRangeMatchesTheVerifiedGeometry() {
        // With the ball exit at the rear face and the robot against the audience wall the exit is 17.8 in from
        // the wall, so the farthest in-line range is 55.52 - 17.8 = 37.7 in (hive_tags.verified.md).
        ScoringPlanner.Config wide = ScoringPlanner.Config.fromConstants(table);
        wide.rangeMinIn = 36.0;
        wide.rangeMaxIn = 60.0;
        wide.coneHalfAngleDeg = 0.0;
        ScoringPlanner p = new ScoringPlanner(field, wide);
        ScoringPlanner.Solution s = p.solve(Pose2.ofDegrees(58.0, 9.2, -90), field.cell(Alliance.RED, Side.AUDIENCE));
        assertTrue(s.ok());
        assertTrue("range " + s.rangeIn, s.rangeIn < 38.0);
    }

    /**
     * The team's one measured good shot: robot front against the audience wall, in line with the HIVE. The ball
     * exit is then exactly the verified wall-limit range from the aim point, and the shot table is anchored there.
     */
    @Test
    public void theRobotFrontAgainstTheWallInLineIsTheMeasuredAnchorShot() {
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 wallShot = new Pose2(cell.axisX, cfg.robotLengthIn / 2.0, -Math.PI / 2.0);
        double range = ShotGeometry.rangeTo(wallShot, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        assertEquals(37.7, range, 0.05);
        assertEquals(1200.0, table.velocityAt(37.7), EPS);
        assertEquals(0.0, Math.toDegrees(ShotGeometry.azimuthOffAxis(wallShot, cfg.launchForwardIn, cfg.launchLeftIn, cell)), 1e-6);
        // Starting there, the planner has nothing to do but a few inches of settling.
        ScoringPlanner.Solution s = planner.solve(wallShot, cell);
        assertTrue(s.ok());
        assertTrue("travel " + s.travelIn, s.travelIn < 2.5);   // a robot touching the wall is nudged off it by the wall clearance
        assertEquals(37.0, s.rangeIn, 1.1);
    }

    @Test
    public void footprintBoxGrowsWhenTheRobotIsTurned() {
        assertEquals(17.8, ShotGeometry.footprintExtentX(17.8, 17.8, Math.toRadians(-90)), 1e-9);
        assertEquals(17.8 * (Math.cos(Math.toRadians(30)) + Math.sin(Math.toRadians(30))),
                ShotGeometry.footprintExtentX(17.8, 17.8, Math.toRadians(30)), 1e-9);
        assertEquals(lengthIn(), cfg.robotLengthIn, EPS);
    }
}
