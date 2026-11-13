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
 * Tests that the TeleOp auto-drive never plans a route that clips the field wall, the HIVE frame or a FLOWER plate, even
 * when the robot starts pressed against a wall or in a corner, at any heading.
 *
 * <p>The bug this guards against: the planner used to plan the TRAVEL part of a route for a single dot (the robot's
 * center) and only checked the real 17.8 in square at the standing pose. But the robot turns while it drives
 * (see {@link HeadingProfile}), and a turning square needs more room than a dot: turned 25 degrees it needs 11.8 in
 * from its center to a wall, not 8.9 in. Starting against a wall and turning toward the HIVE swung a corner through
 * the wall. Now every route is checked with the real rotated robot ({@link RouteSweep}) and repaired if needed.
 *
 * <p>How it works: no robot. For every wall, every corner, many headings and both alliances the planner is asked for
 * a route to each of its two cells. Every answer is then walked in tiny steps by {@link BruteForceFootprint}, a slow
 * checker that shares no code with the planner's check, and must not clip anything by more than the check's own
 * tolerance (a few hundredths of an inch). A second planner, with the new check switched off, is run over the same
 * starts to show that the test would have failed without the change.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*ScoringPlannerWallSafetyTest"} to
 * run only this class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Clip:</b> some part of the robot's rectangle goes through a wall, the HIVE frame or a FLOWER plate.</li>
 *   <li><b>Turn window:</b> the part of a route during which the robot turns; before it the robot keeps its heading,
 *       after it the robot already faces the final way.</li>
 *   <li><b>Repair:</b> what the planner does to a route that would clip: turn only where there is room, or make a
 *       detour to a place with room, or give up on that candidate pose.</li>
 * </ul>
 */
public class ScoringPlannerWallSafetyTest {
    private final FieldModel field = new FieldModel();
    private final ShotTable table = ShotTable.fromConstants();
    private final ScoringPlanner.Config cfg = ScoringPlanner.Config.fromConstants(table);
    private final ScoringPlanner planner = new ScoringPlanner(field, cfg);
    /** Where the robot's edges lie around its center of rotation (front 9.77 in, rear 8.03 in, sides 8.81 in). */
    private final RobotShape robot = cfg.robot;
    /** How much overlap the referee forgives: the planner's own tolerance plus a rounding hair. */
    private final double tol = BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN + 1e-6;

    /** The route a solution tells the drive to follow, as the drive will build it. */
    private static HeadingProfile driven(Pose2 start, ScoringPlanner.Solution s) {
        return HeadingProfile.build(start, s.waypoints, s.pose.heading, s.turnHoldIn, s.turnSettleIn);
    }

    /** The slow referee's verdict on a solution: null if the robot never clips anything, else what it clips. */
    private String clip(Pose2 start, ScoringPlanner.Solution s) {
        return BruteForceFootprint.firstClip(driven(start, s), robot, field.fieldRect(), field.hardObstacles(), tol);
    }

    // ------------------------------------------------------------------ the starting poses (see WallStarts)

    /** A robot against a wall: 0 audience, 1 rear, 2 red, 3 blue; {@code gapIn} inches off it (0 = touching). */
    private Pose2 againstWall(int wall, double along, double headingDeg, double gapIn) {
        return WallStarts.againstWall(field, robot, wall, along, headingDeg, gapIn);
    }

    /** Every start next to a wall or in a corner: 12 headings, touching and 1.5 in off, 3 places along each wall. */
    private List<Pose2> allWallStarts() {
        return WallStarts.all(field, robot);
    }

    // ------------------------------------------------------------------ tests

    /**
     * Situation: 384 starts (against each of the four walls in three places and in each corner, at 12 headings, touching
     *     the wall and 1.5 in off it), for both alliances, planning to each of the alliance's two cells: 1536 plans.
     * Expected: every plan that comes back OK is a route the slow referee finds clear of every wall, the HIVE frame and
     *     every FLOWER plate (within the planner's tolerance), AND still ends at a legal standing pose in its cell's
     *     cone. At least 90% of the plans come back OK (the planner does not just refuse everything), and some of them
     *     needed a repair (a turn window or a detour), so the repair code is really exercised.
     * Why: this is the guarantee the TeleOp relies on: whatever the planner returns, the robot can drive without clipping.
     */
    @Test
    public void everyRouteFromEveryWallAndCornerIsClearOfTheWallsTheHiveAndTheFlowers() {
        int plans = 0;
        int ok = 0;
        int repaired = 0;
        List<String> clips = new ArrayList<String>();
        for (Alliance alliance : Alliance.values()) {
            for (Pose2 start : allWallStarts()) {
                for (Side side : Side.values()) {
                    CellSpec cell = field.cell(alliance, side);
                    ScoringPlanner.Solution s = planner.solve(start, cell);
                    plans++;
                    if (!s.ok()) {
                        continue;
                    }
                    ok++;
                    if (s.turnHoldIn > 0.0 || s.turnSettleIn > 0.0) {
                        repaired++;
                    }
                    assertTrue("must end on a legal standing pose: " + s, planner.standingPoseFree(s.pose));
                    String why = clip(start, s);
                    if (why != null && clips.size() < 5) {
                        clips.add("start " + start + " to " + cell + ": " + why);
                    }
                }
            }
        }
        assertTrue("routes that clip: " + clips, clips.isEmpty());
        assertEquals(1536, plans);
        assertTrue("planner refused too many starts: " + ok + " of " + plans, ok >= 0.9 * plans);
        assertTrue("the repair code never ran (" + repaired + ")", repaired > 0);
    }

    /**
     * Situation: 600 random starts anywhere on the field (a fixed seed, so the run is the same every time): any position
     *     that is clear of the HIVE frame and the FLOWER plates, any heading, both alliances, each asked for both of its
     *     cells. Most of them are nowhere near a wall; some are right beside the HIVE or a FLOWER.
     * Expected: every plan that comes back OK is clear of the walls, the HIVE and the FLOWERS according to the slow
     *     referee; at least 95% come back OK.
     * Why: the walls are not the only thing a turning square can hit. This covers the HIVE frame and the FLOWER plates
     *     with starts the hand-made list does not think of.
     */
    @Test
    public void randomStartsAnywhereOnTheFieldNeverClip() {
        java.util.Random rng = new java.util.Random(7);
        int plans = 0;
        int ok = 0;
        List<String> clips = new ArrayList<String>();
        while (plans < 1200) {
            Pose2 start = new Pose2(9.0 + rng.nextDouble() * (field.size() - 18.0),
                    9.0 + rng.nextDouble() * (field.size() - 18.0), (rng.nextDouble() - 0.5) * 2.0 * Math.PI);
            if (!planner.standingPoseFree(start)) {
                continue;                       // inside the HIVE frame or a FLOWER plate: not a place a robot can be
            }
            for (Alliance alliance : Alliance.values()) {
                CellSpec cell = field.cell(alliance, rng.nextBoolean() ? Side.AUDIENCE : Side.REAR);
                ScoringPlanner.Solution s = planner.solve(start, cell);
                plans++;
                if (!s.ok()) {
                    continue;
                }
                ok++;
                String why = clip(start, s);
                if (why != null && clips.size() < 5) {
                    clips.add("start " + start + " to " + cell + ": " + why);
                }
            }
        }
        assertTrue("routes that clip: " + clips, clips.isEmpty());
        assertTrue("planner refused too many starts: " + ok + " of " + plans, ok >= 0.95 * plans);
    }

    /**
     * Situation: the SAME 1536 plans, made by a planner whose wall-safety check is switched off (its tolerance is made
     *     enormous, so every route "passes"): the behaviour from before the fix.
     * Expected: the slow referee finds at least 20 of those routes clipping a wall, the HIVE or a FLOWER.
     * Why: this is the negative test. It proves the referee can see the bug, so the clean result above is meaningful, and
     *     that the test would fail without the change.
     */
    @Test
    public void withoutTheCheckTheSameStartsDoClip() {
        ScoringPlanner.Config old = ScoringPlanner.Config.fromConstants(table);
        old.sweepToleranceIn = 1.0e6;
        ScoringPlanner unchecked = new ScoringPlanner(field, old);
        int clipping = 0;
        for (Alliance alliance : Alliance.values()) {
            for (Pose2 start : allWallStarts()) {
                for (Side side : Side.values()) {
                    ScoringPlanner.Solution s = unchecked.solve(start, field.cell(alliance, side));
                    if (s.ok() && clip(start, s) != null) {
                        clipping++;
                    }
                }
            }
        }
        assertTrue("only " + clipping + " clipping routes without the check", clipping >= 20);
    }

    /**
     * Situation: a robot standing 0.3 to 1.5 in off the audience wall, at 4 headings, at 21 places along the wall around the shooting spot
     *     (x 57 to 59 in), RED, asked for the audience cell. The shooting lattice has a pose every 1 in, so many of these robots stand
     *     less than 0.5 in from a candidate pose: the planned route is then SHORTER than the 0.5 in limit and has no legs, but it still has
     *     to turn the robot (the drive spins it in place by holding the pose).
     * Expected: every plan that comes back OK is clear of the walls according to the slow referee, which also walks the in-place spin of an
     *     empty route.
     * Why: the planner calls a plan wall-safe when the sweep says so, and the sweep used to treat an empty route as "the robot does not
     *     move". PedroDrive does move it: it turns it in place, and a few percent of the starts below clipped the audience wall by up to 2.6 in.
     * Negative twin: the planner with the check switched off still plans some of those starts as an EMPTY route that turns the robot more
     *     than 25 degrees and clips the wall (the referee sees it), so the clean result above means something.
     */
    @Test
    public void aShortRouteThatTurnsTheRobotNextToTheWallIsNeverPlannedUnlessTheSpinIsClear() {
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        ScoringPlanner.Config offCfg = ScoringPlanner.Config.fromConstants(table);
        offCfg.sweepToleranceIn = 1.0e6;
        ScoringPlanner unchecked = new ScoringPlanner(field, offCfg);
        int emptySpinsThatClipWithoutTheCheck = 0;
        int plans = 0;
        List<String> clips = new ArrayList<String>();
        double[] gaps = {0.3, 0.7, 1.5};
        double[] headings = {0.0, 15.0, 90.0, 180.0};
        for (double gap : gaps) {
            for (double heading : headings) {
                for (int i = 0; i <= 20; i++) {
                    Pose2 start = againstWall(0, 57.0 + 0.1 * i, heading, gap);
                    if (!planner.standingPoseFree(start)) {
                        continue;
                    }
                    ScoringPlanner.Solution s = planner.solve(start, cell);
                    if (s.ok()) {
                        plans++;
                        String why = clip(start, s);
                        if (why != null && clips.size() < 5) {
                            clips.add("start " + start + ": " + why);
                        }
                    }
                    ScoringPlanner.Solution off = unchecked.solve(start, cell);
                    if (off.ok()) {
                        HeadingProfile p = driven(start, off);
                        if (p.isEmpty() && Math.toDegrees(Math.abs(p.turnInPlaceRad())) > BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_DEG
                                && clip(start, off) != null) {
                            emptySpinsThatClipWithoutTheCheck++;
                        }
                    }
                }
            }
        }
        assertTrue("plans " + plans, plans > 100);
        assertTrue("routes that clip: " + clips, clips.isEmpty());
        assertTrue("without the check some starts are planned as an empty route that spins the robot through the wall ("
                + emptySpinsThatClipWithoutTheCheck + ")", emptySpinsThatClipWithoutTheCheck > 0);
    }

    /**
     * Situation: the TeleOp default start (the center of tile C1: x 58.924, y 9.77, facing -90, front against the audience
     *     wall) and the anchor shot (front against the wall, exactly in line with the HIVE), each asking for
     *     the RED audience cell, which is the cell that is up at the start of the match.
     * Expected: both plan, both with no turn window and no detour (the robot only needs to settle a few inches), the
     *     travel is under 2.5 in, the range is the wall's range (the aim point minus the distance of the ball exit from the wall, 39.0 in)
     *     or a little shorter (never longer than the wall allows, never more than 2.5 in short of it), and the slow referee finds
     *     nothing clipped.
     * Why: the new check must not break the anchor shot the team saw work, or the start every TeleOp match begins from.
     *     A robot that is already facing the way it will shoot has no turn to hide, so it must not be sent on a detour.
     */
    @Test
    public void theDefaultStartAndTheAnchorShotPlanWithoutAnyRepair() {
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 defaultStart = Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN,
                BioBuzzConstants.Controls.DEFAULT_START_Y_IN, BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG);
        Pose2 anchor = new Pose2(cell.axisX, robot.frontIn(), -0.5 * Math.PI);
        for (Pose2 start : new Pose2[]{defaultStart, anchor}) {
            ScoringPlanner.Solution s = planner.solve(start, cell);
            assertTrue("must plan from " + start + ": " + s, s.ok());
            assertEquals("no turn window needed from " + start, 0.0, s.turnHoldIn, 1e-9);
            assertEquals("no turn window needed from " + start, 0.0, s.turnSettleIn, 1e-9);
            assertTrue("travel " + s.travelIn, s.travelIn < 2.5);
            // The audience wall limits the range to the aim point minus the ball exit's distance from the wall (front extent plus the
            // exit's distance behind the center of rotation); the planner searches in 1 in steps and keeps off the wall.
            double wallRange = cell.originY - (robot.frontIn() - BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN);
            assertTrue("range " + s.rangeIn + " against the wall's " + wallRange, s.rangeIn <= wallRange + 0.05 && s.rangeIn >= wallRange - 2.5);
            assertNull("clipped: " + clip(start, s), clip(start, s));
        }
    }

    /**
     * Situation: a RED robot standing against the audience wall, touching it, right below the middle of the HIVE
     *     (x = the field center, 70.674, so the open floor straight ahead is the HIVE itself), facing +90 degrees (front toward the field,
     *     back toward the wall) and asked for the audience cell, which it shoots at facing -90. That is a 180 degree
     *     flip, and a rectangle flipping next to a wall swings its corners through 13.2 in of room.
     * Expected: the planner returns a route; the robot keeps its heading for the first few inches (hold above 0), so it
     *     leaves the wall before it turns; the first waypoint is farther from the wall than the start; the slow referee
     *     finds nothing clipped. The same start with the check switched off clips the wall.
     * Why: this is the reviewer's case in its worst form. Before, the route was a straight 2 in move with the whole flip
     *     done on top of the wall.
     */
    @Test
    public void aRobotFlippingAgainstTheWallGoesOutToOpenFloorBeforeItTurns() {
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 start = againstWall(0, BioBuzzConstants.Field.CENTER_IN, 90.0, 0.0);
        ScoringPlanner.Solution s = planner.solve(start, cell);
        assertTrue("must plan: " + s, s.ok());
        assertTrue("hold " + s.turnHoldIn, s.turnHoldIn > 1.0);
        assertTrue("first waypoint y " + s.waypoints.get(0)[1], s.waypoints.get(0)[1] > start.y + 3.0);
        assertTrue(driven(start, s).maxTurnRateDegPerIn() <= cfg.maxTurnDegPerIn + 1e-6);
        assertNull("clipped: " + clip(start, s), clip(start, s));

        ScoringPlanner.Config off = ScoringPlanner.Config.fromConstants(table);
        off.sweepToleranceIn = 1.0e6;
        ScoringPlanner.Solution unchecked = new ScoringPlanner(field, off).solve(start, cell);
        assertTrue(unchecked.ok());
        assertNotNull("the unchecked route must clip, or this test proves nothing", clip(start, unchecked));
    }

    /**
     * Situation: the same flip, but the planner is told the robot can hardly turn while it drives (1/1000 degree per
     *     inch), so no turn window or detour can ever be gentle enough.
     * Expected: NO_FEASIBLE_POSE, which makes the controller stop with a fault instead of driving. With the real setting
     *     the same start plans.
     * Why: this is the "if nothing safe exists the candidate is rejected like any blocked pose" rule. A planner that
     *     fell back to the unchecked route here would fail this test.
     */
    @Test
    public void whenNoSafeRepairExistsTheCandidateIsRejected() {
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 start = againstWall(0, BioBuzzConstants.Field.CENTER_IN, 90.0, 0.0);
        assertTrue(planner.solve(start, cell).ok());
        ScoringPlanner.Config stiff = ScoringPlanner.Config.fromConstants(table);
        stiff.maxTurnDegPerIn = 0.001;
        ScoringPlanner.Solution s = new ScoringPlanner(field, stiff).solve(start, cell);
        assertEquals(ScoringPlanner.Status.NO_FEASIBLE_POSE, s.status);
    }

    /**
     * Situation: every start of the big sweep, for the RED robot and the RED cells only.
     * Expected: whenever the planner gives a route a turn window (a hold or a settle), the turn inside it is never
     *     faster than the limit the robot can follow (PATH_MAX_TURN_DEG_PER_IN, degrees per inch).
     * Why: a window that squeezed 180 degrees into a few inches would pass the geometry check on paper and then be
     *     impossible for the real robot, which would lag behind its heading and swing a corner into the wall.
     */
    @Test
    public void aSqueezedTurnIsNeverFasterThanTheRobotCanFollow() {
        int windowed = 0;
        for (Pose2 start : allWallStarts()) {
            for (Side side : Side.values()) {
                ScoringPlanner.Solution s = planner.solve(start, field.cell(Alliance.RED, side));
                if (s.ok() && (s.turnHoldIn > 0.0 || s.turnSettleIn > 0.0)) {
                    windowed++;
                    assertTrue("turn rate " + driven(start, s).maxTurnRateDegPerIn() + " from " + start,
                            driven(start, s).maxTurnRateDegPerIn() <= cfg.maxTurnDegPerIn + 1e-6);
                }
            }
        }
        assertTrue("no window was ever used (" + windowed + ")", windowed > 0);
    }

    /**
     * Situation: a RED robot and its half-turn image (a BLUE robot), both against a wall and flipping, each asked for
     *     its own alliance's audience-side cell.
     * Expected: the BLUE plan is the half-turn of the RED plan: the same standing pose rotated half a turn about the
     *     field center, the same hold and settle distances.
     * Why: the numbers are written once for RED. The wall-safety repairs must not favour one alliance.
     */
    @Test
    public void theBlueRepairIsTheHalfTurnOfTheRedRepair() {
        Pose2 red = againstWall(0, 40.0, 150.0, 0.0);
        ScoringPlanner.Solution sr = planner.solve(red, field.cell(Alliance.RED, Side.AUDIENCE));
        Pose2 blue = Alliance.BLUE.fromRed(red, field.size());
        ScoringPlanner.Solution sb = planner.solve(blue, field.cell(Alliance.BLUE, Side.REAR));
        assertTrue(sr.ok() && sb.ok());
        Pose2 expected = Alliance.BLUE.fromRed(sr.pose, field.size());
        assertEquals(expected.x, sb.pose.x, 1e-6);
        assertEquals(expected.y, sb.pose.y, 1e-6);
        assertEquals(sr.turnHoldIn, sb.turnHoldIn, 1e-6);
        assertEquals(sr.turnSettleIn, sb.turnSettleIn, 1e-6);
        assertEquals(sr.waypoints.size(), sb.waypoints.size());
    }
}
