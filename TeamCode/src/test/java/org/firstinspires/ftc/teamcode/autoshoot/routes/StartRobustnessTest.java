package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.junit.Test;

/**
 * "What if the robot is placed an inch off, or a few degrees turned?" ({@link StartRobustness}): how a start is moved (the wall stops it, so only the
 * part along the wall and the turn are left), what the report finds, and what it says about every route of the catalog.
 */
public class StartRobustnessTest {
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double SIDE = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
    private static final FieldModel MODEL = new FieldModel();
    private static final AllianceField RED = new AllianceField(MODEL, Alliance.RED);
    private static final double M_IN = BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN;
    private static final double M_DEG = BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_DEG;

    // ---- how a start is moved ------------------------------------------------------------------------------------

    /**
     * Situation: a robot on the audience wall, front first, 1 in and 3 degrees off every way.
     * Expected: the wall takes away the part of the move that goes toward or away from it, so the robot slides along the wall and turns: 3 x positions and 3 headings,
     *     8 different places besides the start. Each is pushed back against the wall exactly (its footprint touches it), and the start itself is not one of them.
     * Why: a robot that is 1 in off a wall is not a legal start at all (G304.C), so "an inch off" must mean "an inch along the wall".
     */
    @Test
    public void aStartOnAWallSlidesAlongItAndTurnsAndStaysAgainstTheWall() {
        Pose2 start = Pose2.ofDegrees(58.0, FRONT, -90);
        List<StartRobustness.Move> moves = StartRobustness.moves(RED, start, M_IN, M_DEG);
        assertEquals(moves.toString(), 8, moves.size());
        boolean turned = false;
        boolean slid = false;
        for (StartRobustness.Move m : moves) {
            Rect b = Footprint.of(m.start).bounds();
            assertEquals(m.describe() + " touches the audience wall", 0.0, b.minY, 1e-9);
            assertFalse(m.describe() + " is the start", Math.abs(m.start.x - start.x) < 1e-9 && Math.abs(m.start.heading - start.heading) < 1e-9);
            turned |= Math.abs(m.dHeadingDeg) > 0.0;
            slid |= Math.abs(m.start.x - start.x) > 0.5;
            assertTrue("the move is no bigger than the margin along the wall", Math.abs(m.start.x - start.x) <= M_IN + 1e-9);
        }
        assertTrue(turned && slid);
    }

    /**
     * Situation: a robot in the audience/red corner (it touches both walls) and a robot touching no wall at all.
     * Expected: the corner robot can only turn (2 moves, both still in the corner, touching both walls); the robot away from every wall has nothing to push it back,
     *     so all 26 combinations of the margin are different places.
     */
    @Test
    public void aCornerStartOnlyTurnsAndAStartAwayFromTheWallsMovesEveryWay() {
        Pose2 corner = Pose2.ofDegrees(FRONT, SIDE, 180);
        List<StartRobustness.Move> inCorner = StartRobustness.moves(RED, corner, M_IN, M_DEG);
        assertEquals(inCorner.toString(), 2, inCorner.size());
        for (StartRobustness.Move m : inCorner) {
            Rect b = Footprint.of(m.start).bounds();
            assertEquals(0.0, b.minX, 1e-9);
            assertEquals(0.0, b.minY, 1e-9);
        }
        assertEquals(26, StartRobustness.moves(RED, Pose2.ofDegrees(35.0, 20.0, -90), M_IN, M_DEG).size());
    }

    /** Situation: the margin 0. Expected: nothing moves, so there are no moves. */
    @Test
    public void aMarginOfNothingMovesNothing() {
        assertEquals(0, StartRobustness.moves(RED, Pose2.ofDegrees(58.0, FRONT, -90), 0.0, 0.0).size());
    }

    /**
     * Situation: a robot on the red wall facing it, 0.5 in and 2.0 in beyond the end of the bracket of the red-wall FLOWER; the placement margin and much smaller ones.
     * Expected: with the real margin the first is too close (an inch nearer it is on the FLOWER) and the second is fine; with a margin of 0.2 in both pass.
     */
    @Test
    public void aStartTooCloseToAFlowerFailsTheQuickCheckAndOneWithRoomPasses() {
        double bracketEnd = RED.flowerPlate(0).maxY + 0.36;           // the bracket, which the search also keeps clear of
        Pose2 close = Pose2.ofDegrees(FRONT, bracketEnd + SIDE + 0.5, 180);
        Pose2 roomy = Pose2.ofDegrees(FRONT, bracketEnd + SIDE + 2.0, 180);
        assertTrue(StartRules.isLegalAndClear(RED, close));
        assertFalse(StartRobustness.startsStayLegal(RED, close, M_IN, M_DEG));
        assertTrue(StartRobustness.startsStayLegal(RED, roomy, M_IN, M_DEG));
        assertTrue(StartRobustness.startsStayLegal(RED, close, 0.2, 0.0));

    }

    // ---- the report ---------------------------------------------------------------------------------------------------------

    private static AutoRoute.Builder partnerStyle(double x) {
        return AutoRoute.builder("margin").partnerTipsFirst().start(x, SIZE - FRONT, 90)
                .waitForTip(16.0).shootUp(x, 130.0, 90, 4)
                .driveConstant(x, 112.0, 90).driveLinear(12.0, 108.0, 0.0).park(12.0, 108.0, 0.0);
    }

    /**
     * Situation: the partner-family route's start at x = 61.5 (the robot's edge is 0.27 in from the center line), and a route that starts on the audience
     *     wall in the middle of the red half (room all round).
     * Expected: the first is legal as written but moving it 1 in puts it into the other half: the report finds a new START ERROR and says the placement is not
     *     safe. The second passes every placement. The time spread is not negative.
     */
    @Test
    public void aStartNextToTheCenterLineIsFlaggedAndOneWithRoomIsNot() {
        StartRobustness.Report tight = StartRobustness.check(partnerStyle(61.5).build());
        assertTrue("legal as written", tight.nominalErrors.isEmpty());
        assertFalse("but not when it is an inch off: " + tight.summary(), tight.placementSafe());
        boolean startRule = false;
        for (Issue i : tight.newErrors()) {
            startRule |= i.rule == Rule.START_G304 && i.message.contains("opponent half");
        }
        assertTrue(tight.newErrors().toString(), startRule);
        assertTrue(tight.failedCases().size() >= 1);
        for (StartRobustness.Case c : tight.failedCases()) {
            assertTrue(c.move.describe(), c.failed());
        }

        StartRobustness.Report roomy = StartRobustness.check(RouteAnalysisTest.corridorPark().build());
        assertTrue(roomy.nominalErrors.toString(), roomy.nominalErrors.isEmpty());
        assertTrue(roomy.summary() + roomy.newErrors(), roomy.placementSafe());
        assertTrue(roomy.newErrors().isEmpty());
        assertTrue("some placements were tried", roomy.cases.size() >= 8);
        assertTrue(roomy.spreadSeconds() >= 0.0);
        assertEquals(roomy.maxSeconds() - roomy.minSeconds(), roomy.spreadSeconds(), 1e-12);
    }

    /**
     * Situation: a robot facing the rear wall between the rear-wall FLOWER and the center line, the one place where it could shoot into the rear cell without
     *     driving. The gap is 20.42 in wide (the plate ends at x 50.26, the center line is at 70.67); the robot is 17.80 in wide.
     * Expected: with the full margin (1 in and 3 degrees together) NO position passes: 17.80 + 2 x (1.0 + 0.51 for the swing of a corner turned 3 degrees) is more
     *     than 20.42. With a smaller margin there is a place that passes (x about 60.5). This is the fact the table of shortcuts has to say plainly.
     */
    @Test
    public void theSlotBetweenTheRearFlowerAndTheCenterLineIsTooNarrowForTheFullMargin() {
        TimingModel timing = new TimingModel();
        RouteTemplate shootFromTheStart = new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 start) {
                return AutoRoute.builder("slot").partnerTipsFirst().start(start.x, start.y, start.headingDegrees())
                        .waitForTip(16.0).shootUp(start.x, start.y, start.headingDegrees(), 4).build();
            }
        };
        for (double x = 58.0; x <= 63.0; x += 0.1) {
            StartRobustness.Report full = StartRobustness.check(shootFromTheStart, Pose2.ofDegrees(x, SIZE - FRONT, 90), Alliance.RED, timing, M_IN, M_DEG);
            assertFalse("x = " + x + " passes the full margin?! " + full.summary(), full.placementSafe());
        }
        StartRobustness.Report easier = StartRobustness.check(shootFromTheStart, Pose2.ofDegrees(60.46, SIZE - FRONT, 90), Alliance.RED, timing, 0.5, 1.0);
        assertTrue(easier.summary() + easier.newErrors(), easier.placementSafe());
    }

    /**
     * Situation: the same two routes (one flagged, one safe) for the BLUE alliance.
     * Expected: the same verdicts and the same number of failed placements (the validator and the half turn are symmetric).
     */
    @Test
    public void theBlueAllianceGetsTheSameVerdicts() {
        for (AutoRoute route : new AutoRoute[] {partnerStyle(61.5).build(), RouteAnalysisTest.corridorPark().build()}) {
            StartRobustness.Report red = StartRobustness.check(route);
            StartRobustness.Report blue = StartRobustness.check(route.forAlliance(Alliance.BLUE));
            assertEquals(route.id() + " cases", red.cases.size(), blue.cases.size());
            assertEquals(route.id() + " failed", red.failedCases().size(), blue.failedCases().size());
            assertEquals(route.id() + " safe", red.placementSafe(), blue.placementSafe());
            assertEquals(route.id() + " time", red.nominalSeconds, blue.nominalSeconds, 1e-9);
        }
    }

    /**
     * Situation: a start with an ERROR of its own (the old OpMode start 0.57 in through the audience wall).
     * Expected: it is not "placement safe", and its own ERRORs are not reported as caused by the placement ({@code newErrors} does not list them).
     */
    @Test
    public void anErrorTheStartAlreadyHasIsNotBlamedOnThePlacement() {
        AutoRoute old = AutoRoute.builder("old").start(35.0, 9.2, -90).waitSeconds(1).build();
        StartRobustness.Report r = StartRobustness.check(old);
        assertFalse(r.nominalErrors.isEmpty());
        assertFalse(r.placementSafe());
        for (Issue i : r.newErrors()) {
            assertFalse("the start's own ERROR is not new: " + i, i.rule == Rule.START_G304 && i.stepIndex == 0);
        }
        boolean ownRule = false;
        for (Issue i : r.nominalErrors) {
            ownRule |= i.rule == Rule.START_G304 && i.stepIndex == 0;
        }
        assertTrue("the start has its own G304 ERROR: " + r.nominalErrors, ownRule);
    }

    /**
     * Situation: a template that lets the route change with the start (the shot is taken from wherever the robot stands), moved by the check.
     * Expected: the route is built again for every placement, so the shot pose follows the start: the report's time spread is more than the move of the start alone
     *     would give with fixed steps, and no placement gives an ERROR.
     */
    @Test
    public void aTemplateIsBuiltAgainForEveryPlacement() {
        final int[] built = {0};
        RouteTemplate shootWhereItStands = new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 start) {
                built[0]++;
                return AutoRoute.builder("t").start(start.x, start.y, start.headingDegrees())
                        .shootUp(start.x, start.y, start.headingDegrees(), 4).driveConstant(30.0, 14.8, -90).build();
            }
        };
        Pose2 start = Pose2.ofDegrees(58.0, FRONT, -90);
        StartRobustness.Report r = StartRobustness.check(shootWhereItStands, start, Alliance.RED, new TimingModel(), M_IN, M_DEG);
        assertEquals("the start itself and every placement", 1 + r.cases.size(), built[0]);
        assertTrue(r.cases.size() >= 8);
        for (StartRobustness.Case c : r.cases) {
            assertTrue(c.move.describe() + " " + c.errors, !c.failed());
        }
    }

    // ---- the catalog ---------------------------------------------------------------------------------------------------------

    /**
     * What the placement check says about the routes of the catalog TODAY: the rules of the ERRORs that appear only because the robot is moved. This is a
     * report, written down so a change to a route or to the margin is seen; a route that is fixed is deleted from this table in the same change.
     * <ul>
     *   <li>{@code red_c1}: its start is 0.57 in through the audience wall (the old OpMode's number), so the first move to the shooting pose already clips the wall;
     *       moved 1 in the clip is a little deeper (a new FIELD_BOUNDS ERROR at step 2).</li>
     *   <li>{@code partner_rear_shoot_park}: its start x = 61.5 leaves only 0.27 in between the robot's edge and the center line. A hand that is 1 in off puts
     *       the robot into the other alliance's half (G304.A): the START rule, not the route, breaks.</li>
     * </ul>
     */
    private static Map<String, Set<Rule>> knownPlacementFindings() {
        Map<String, Set<Rule>> known = new HashMap<String, Set<Rule>>();
        known.put("red_c1", EnumSet.of(Rule.FIELD_BOUNDS));
        known.put("partner_rear_shoot_park", EnumSet.of(Rule.START_G304));
        return known;
    }

    /**
     * Situation: every route of the catalog, for both alliances, moved by the placement margin and validated.
     * Expected: the ERRORs that appear only because of the move are exactly the known ones above (so a new route, or a change to a route, that is placed too
     *     close to something fails here and says which rule); the two alliances agree; the time spread is under a second; the table is printed.
     */
    @Test
    public void theCatalogsPlacementFindingsAreTheKnownOnesAndBothAlliancesAgree() {
        Map<String, Set<Rule>> known = knownPlacementFindings();
        StringBuilder table = new StringBuilder("\nPLACEMENT CHECK of the catalog (the robot set down " + M_IN + " in along the wall and " + M_DEG + " degrees off)\n");
        for (AutoRoute route : RouteCatalog.all()) {
            StartRobustness.Report red = StartRobustness.check(route);
            StartRobustness.Report blue = StartRobustness.check(route.forAlliance(Alliance.BLUE));
            table.append(String.format(java.util.Locale.ROOT, "  %-26s start %s  own ERRORs %d  %s%n", route.id(), route.startPose(), red.nominalErrors.size(), red.summary()));
            Set<Rule> found = EnumSet.noneOf(Rule.class);
            for (Issue i : red.newErrors()) {
                found.add(i.rule);
                table.append("      new ERROR: ").append(i.rule).append(", step ").append(i.stepNumber()).append(": ").append(i.message).append('\n');
            }
            Set<Rule> expected = known.containsKey(route.id()) ? known.get(route.id()) : EnumSet.noneOf(Rule.class);
            assertEquals(route.id() + ": the rules of the ERRORs that only the placement causes\n" + table, expected, found);
            assertEquals(route.id() + " red and blue agree on the number of placements", red.cases.size(), blue.cases.size());
            assertEquals(route.id() + " red and blue agree on how many fail", red.failedCases().size(), blue.failedCases().size());
            assertEquals(route.id() + " red and blue agree on the time", red.nominalSeconds, blue.nominalSeconds, 1e-9);
            assertEquals(route.id() + " red and blue agree on the spread", red.spreadSeconds(), blue.spreadSeconds(), 1e-9);
            assertTrue(route.id() + " spread " + red.spreadSeconds(), red.spreadSeconds() >= 0.0 && red.spreadSeconds() < 1.0);
            // a route with no ERROR of its own and no finding of the placement has the margin
            assertEquals(route.id(), red.nominalErrors.isEmpty() && found.isEmpty(), red.placementSafe());
        }
        System.out.println(table);
    }
}
