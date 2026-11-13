package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Locale;
import java.util.Random;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The TeleOp planner and the lane under the HIVE ({@code AutoShoot.ALLOW_UNDER_HIVE}). By default the planner goes AROUND the whole HIVE footprint
 * (G409 / G417: never be under a HIVE that is tipping). With the flag on, only the foot bars and the leg stubs are solid, so the way from the
 * audience side to the rear side may go along the lane between the feet, about 20 in shorter; if that way cannot be driven safely the planner
 * falls back to the way around.
 *
 * <p>How it works: no robot. The planner is asked for the shooting pose in front of the red REAR cell from the C1 start on the audience side, with
 * the flag off and on. Every pose of the route it returns is then walked (the robot turns the way the plan says) and the real rectangle is
 * compared with the exact HIVE solid by a separate, simple check.
 *
 * <p>Words used: <b>lane</b> is the open floor between the foot bars; <b>margin</b> is the space the planner keeps from an obstacle beyond the
 * robot's own size (1.5 in); <b>route</b> is the list of waypoints the planner returns.
 */
public class PlannerLaneTest {
    private final FieldModel field = new FieldModel();
    private final ShotTable table = ShotTable.fromConstants();

    private ScoringPlanner planner(boolean allowUnderHive) {
        ScoringPlanner.Config cfg = ScoringPlanner.Config.fromConstants(table);
        cfg.allowUnderHive = allowUnderHive;
        return new ScoringPlanner(field, cfg);
    }

    private static Pose2 c1() {
        return new Pose2(BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN + 0.076, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -Math.PI / 2.0);
    }

    /** The smallest gap, over the whole planned route, between the robot's rectangle and the exact HIVE solid. */
    private double smallestGapToTheSolid(Pose2 start, ScoringPlanner.Solution s) {
        HeadingProfile profile = HeadingProfile.build(start, s.waypoints, s.pose.heading, s.turnHoldIn, s.turnSettleIn);
        HiveSolid solid = field.hiveSolid();
        double worst = Double.POSITIVE_INFINITY;
        for (double d = 0.0; d <= profile.length() + 1e-9; d += 0.25) {
            worst = Math.min(worst, solid.signedGap(Footprint.of(profile.poseAt(Math.min(d, profile.length())))));
        }
        return worst;
    }

    /** The smallest gap between the robot's rectangle and the whole HIVE footprint box over the route (negative = inside the box). */
    private double smallestGapToTheFrame(Pose2 start, ScoringPlanner.Solution s) {
        HeadingProfile profile = HeadingProfile.build(start, s.waypoints, s.pose.heading, s.turnHoldIn, s.turnSettleIn);
        double worst = Double.POSITIVE_INFINITY;
        for (double d = 0.0; d <= profile.length() + 1e-9; d += 0.25) {
            worst = Math.min(worst, Footprint.of(profile.poseAt(Math.min(d, profile.length()))).signedGap(field.hiveFrame()));
        }
        return worst;
    }

    /**
     * Situation: the flag off (the default), from the C1 start on the audience side to the red REAR cell.
     * Expected: a good solution whose route never comes within the margin of the whole HIVE footprint: it goes around.
     * Negative twin: the shipped constant is false.
     */
    @Test
    public void byDefaultThePlannerGoesAroundTheWholeFrame() {
        assertFalse("the lane is off by default", BioBuzzConstants.AutoShoot.ALLOW_UNDER_HIVE);
        assertFalse(ScoringPlanner.Config.fromConstants(table).allowUnderHive);
        ScoringPlanner.Solution s = planner(false).solve(c1(), field.cell(Alliance.RED, Side.REAR));
        assertTrue(s.toString(), s.ok());
        assertTrue("goes around the frame: " + smallestGapToTheFrame(c1(), s), smallestGapToTheFrame(c1(), s) >= BioBuzzConstants.AutoShoot.KEEPOUT_MARGIN_IN - 0.06);
    }

    /**
     * Situation: the flag on, the same trip.
     * Expected: a good solution that is more than 10 in shorter than the way around; the route really passes between the foot bars (it comes
     *     within the footprint box of the HIVE), and at every pose keeps the planner's margin from the exact solid; the pose it ends at is the same
     *     kind of place (in front of the rear cell, back to the cell).
     * Why: the lane is the point of the flag. The test prints the two lengths quoted in docs/AUTOS.md.
     */
    @Test
    public void withTheFlagTheRouteGoesThroughTheLaneAndIsShorter() {
        Pose2 start = c1();
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        ScoringPlanner.Solution around = planner(false).solve(start, rear);
        ScoringPlanner.Solution lane = planner(true).solve(start, rear);
        assertTrue(around.toString(), around.ok());
        assertTrue(lane.toString(), lane.ok());
        System.out.println(String.format(Locale.ROOT, "PLANNER lane vs around (C1 start to the red REAR cell): lane %.1f in, around %.1f in, saves %.1f in",
                lane.travelIn, around.travelIn, around.travelIn - lane.travelIn));
        assertTrue("saves " + (around.travelIn - lane.travelIn) + " in", around.travelIn - lane.travelIn > 10.0);
        assertTrue("through the footprint box", smallestGapToTheFrame(start, lane) < 0.0);
        assertTrue("with the margin to the solid: " + smallestGapToTheSolid(start, lane),
                smallestGapToTheSolid(start, lane) >= BioBuzzConstants.AutoShoot.KEEPOUT_MARGIN_IN - 0.06);
        assertEquals(90.0, lane.pose.headingDegrees(), 30.0);
        assertTrue(lane.pose.y > field.hiveFrame().maxY);
    }

    /**
     * Situation: the flag on, but a trip that has nothing to do with the HIVE's far side: from C1 to the AUDIENCE cell (the UP cell at match
     *     start), which is on the near side.
     * Expected: the same route as without the flag (the lane is never an excuse to change a trip that does not cross the HIVE).
     */
    @Test
    public void aTripThatDoesNotCrossTheHiveIsUnchangedByTheFlag() {
        Pose2 start = c1();
        CellSpec audience = field.cell(Alliance.RED, Side.AUDIENCE);
        ScoringPlanner.Solution off = planner(false).solve(start, audience);
        ScoringPlanner.Solution on = planner(true).solve(start, audience);
        assertTrue(off.ok() && on.ok());
        assertEquals(off.pose.x, on.pose.x, 1e-9);
        assertEquals(off.pose.y, on.pose.y, 1e-9);
        assertEquals(off.travelIn, on.travelIn, 1e-9);
    }

    /**
     * Situation: 60 random starts (seeded) on the audience side of the red half (y 12 to 40), each asked for the rear cell with the flag on.
     * Expected: every solution keeps at least the margin from the exact HIVE solid along its whole route, and a solution is found whenever one is found
     *     with the flag off (the fallback means the lane never makes the planner fail).
     * Why: the same safety as the default, with the shorter way available.
     */
    @Test
    public void randomStartsKeepTheMarginFromTheSolidAndNeverLoseASolution() {
        Random rng = new Random(2027);
        ScoringPlanner on = planner(true);
        ScoringPlanner off = planner(false);
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        int solved = 0;
        for (int i = 0; i < 60; i++) {
            Pose2 start = new Pose2(12.0 + rng.nextDouble() * 50.0, 12.0 + rng.nextDouble() * 28.0, (rng.nextDouble() - 0.5) * 2.0 * Math.PI);
            ScoringPlanner.Solution without = off.solve(start, rear);
            ScoringPlanner.Solution with = on.solve(start, rear);
            if (without.ok()) {
                assertTrue("a solution with the flag whenever there is one without: " + start, with.ok());
            }
            if (with.ok()) {
                solved++;
                assertNotNull(with.waypoints);
                assertTrue("start " + start + ": gap to the solid " + smallestGapToTheSolid(start, with),
                        smallestGapToTheSolid(start, with) >= BioBuzzConstants.AutoShoot.KEEPOUT_MARGIN_IN - 0.06);
            }
        }
        assertTrue("solved " + solved, solved > 30);
    }

    /**
     * Situation: the planner built with the flag on, asked where a robot may STAND: in the middle of the lane, and on a foot bar.
     * Expected: in the lane at a shooting height it is free as far as the HIVE goes (the lane is open); on a foot bar it is not.
     * Negative twin: with the flag off the lane is solid.
     */
    @Test
    public void standingInTheLaneIsFreeOnlyWithTheFlag() {
        Pose2 inLane = new Pose2(BioBuzzConstants.Field.CENTER_IN, BioBuzzConstants.Field.CENTER_IN, Math.PI / 2.0);
        Pose2 onBar = new Pose2(BioBuzzConstants.Field.CENTER_IN - 22.0, BioBuzzConstants.Field.CENTER_IN, Math.PI / 2.0);
        assertTrue(planner(true).standingPoseFree(inLane));
        assertFalse(planner(true).standingPoseFree(onBar));
        assertFalse(planner(false).standingPoseFree(inLane));
        assertFalse(planner(false).standingPoseFree(onBar));
    }

    /**
     * Situation: {@link RouteSweep} given the exact solid as polygons, a robot driving straight up the lane and one driving up a line through the left
     *     foot bar.
     * Expected: the first is safe, the second is a violation that names a solid.
     */
    @Test
    public void theRouteCheckJudgesPolygonsByTheirTrueShape() {
        RouteSweep.Settings settings = RouteSweep.Settings.fromConstants();
        RouteSweep sweep = new RouteSweep(field.fieldRect(), BioBuzzConstants.AutoShoot.WALL_CLEARANCE_IN, field.flowerKeepOuts(), field.hardPolygons(),
                BioBuzzConstants.AutoShoot.KEEPOUT_MARGIN_IN, RobotShape.fromConstants(), settings);
        double x = BioBuzzConstants.Field.CENTER_IN;
        List<double[]> up = new java.util.ArrayList<double[]>();
        up.add(new double[] {x, 110.0});
        assertNull(sweep.check(HeadingProfile.build(new Pose2(x, 30.0, Math.PI / 2.0), up, Math.PI / 2.0)));
        List<double[]> bar = new java.util.ArrayList<double[]>();
        bar.add(new double[] {BioBuzzConstants.Field.CENTER_IN - 22.0, 110.0});
        RouteSweep.Violation v = sweep.check(HeadingProfile.build(new Pose2(BioBuzzConstants.Field.CENTER_IN - 22.0, 30.0, Math.PI / 2.0), bar, Math.PI / 2.0));
        assertNotNull(v);
        assertTrue(v.what, v.what.startsWith("solid"));
    }

}
