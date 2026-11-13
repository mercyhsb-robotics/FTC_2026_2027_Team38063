package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The routes of {@link SelfTipRoutes}: our robot shoots the first tip itself. Each test says what it checks and why; the numbers that the route texts
 * quote (the seconds a start saves, the points) are checked against what the code computes, because a text that quotes a wrong number is worse than none.
 */
public class SelfTipRoutesTest {
    private static final String[] TIER_WORDS = {"Safe", "Standard", "Aggressive", "Advanced"};
    private static final double TIME_LIMIT_SEC = BioBuzzConstants.Auto.AUTO_DURATION_SEC - BioBuzzConstants.Auto.TIME_SAFETY_MARGIN_SEC;

    private static List<AutoRoute> routes() {
        return SelfTipRoutes.routes();
    }

    private static AutoRoute route(String id) {
        for (AutoRoute r : routes()) {
            if (r.id().equals(id)) {
                return r;
            }
        }
        throw new AssertionError("no route " + id);
    }

    private static List<Issue> warnings(AutoRoute r, Rule rule) {
        List<Issue> out = new ArrayList<Issue>();
        for (Issue i : RouteValidator.check(r)) {
            if (i.severity == Severity.WARN && i.rule == rule) {
                out.add(i);
            }
        }
        return out;
    }

    /**
     * Situation: the catalog of the SELF family.
     * Expected: at least 8 routes, each id unique, lowercase letters, digits and underscores only, and none of them clashes with a route written elsewhere.
     * Why: ids become file names and stay in use for ever; the team asked for at least 8 different routes.
     */
    @Test
    public void thereAreAtLeastEightRoutesWithUniqueLowercaseIds() {
        List<AutoRoute> all = routes();
        assertTrue("routes: " + all.size(), all.size() >= 8);
        Set<String> seen = new HashSet<String>();
        Pattern lowercase = Pattern.compile("[a-z][a-z0-9_]*");
        for (AutoRoute r : all) {
            assertTrue("id '" + r.id() + "' is lowercase with underscores", lowercase.matcher(r.id()).matches());
            assertTrue("duplicate id " + r.id(), seen.add(r.id()));
        }
        Set<String> inCatalog = new HashSet<String>();
        for (AutoRoute r : RouteCatalog.all()) {
            assertTrue("duplicate id in the catalog " + r.id(), inCatalog.add(r.id()));
        }
        assertTrue(inCatalog.containsAll(seen));
    }

    /**
     * Situation: every route of the file, for both alliances.
     * Expected: no ERROR. A WARN is allowed only when the note of the step it is about explains it (the note is not empty), and only these kinds: the first volley's spare
     *     (G417), the top-up volley that the plan thinks is at a DOWN cell (G417, explained in the note: the camera decides).
     */
    @Test
    public void everyRouteValidatesWithNoErrorAndEveryWarnIsExplainedByItsStepNote() {
        for (AutoRoute red : routes()) {
            for (Alliance a : Alliance.values()) {
                AutoRoute r = red.forAlliance(a);
                List<Issue> issues = RouteValidator.check(r);
                for (Issue i : issues) {
                    assertFalse(r.id() + " " + a + ": " + i, i.isError());
                    assertEquals(r.id() + " " + a + ": only G417 WARNs are expected: " + i, Rule.SHOOT_TARGET, i.rule);
                    String note = r.step(i.stepIndex).note();
                    assertNotNull(note);
                    assertTrue(r.id() + " " + a + ": the note of step " + (i.stepIndex + 1) + " explains the WARN", note.length() > 80);
                }
            }
        }
    }

    /**
     * Situation: the time the plan estimates for every route.
     * Expected: inside the AUTO limit minus the safety margin, so a route never finishes in the last 2 s, and none is a near-miss: the Safe and Standard routes
     *     have at least 10 s to spare and every route has at least 4 s.
     */
    @Test
    public void everyRouteFinishesInsideTheAutoLimitMinusTheSafetyMarginWithSecondsToSpare() {
        for (AutoRoute r : routes()) {
            double sec = r.estimatedSeconds();
            assertTrue(r.id() + " takes " + sec + " s, limit " + TIME_LIMIT_SEC, sec <= TIME_LIMIT_SEC);
            assertTrue(r.id() + " has only " + (TIME_LIMIT_SEC - sec) + " s to spare", TIME_LIMIT_SEC - sec >= 4.0);
            if (r.title().startsWith("Safe") || r.title().startsWith("Standard")) {
                assertTrue(r.id() + " is " + r.title().split(":")[0] + " and takes " + sec + " s", sec <= BioBuzzConstants.Auto.AUTO_DURATION_SEC - 14.0);
            }
        }
    }

    /**
     * Situation: the titles. Expected: each starts with exactly one of Safe, Standard, Aggressive or Advanced, then a colon, and no other tier word appears before it.
     */
    @Test
    public void everyTitleStartsWithExactlyOneTierWordAndAColon() {
        for (AutoRoute r : routes()) {
            int matches = 0;
            for (String tier : TIER_WORDS) {
                if (r.title().startsWith(tier + ":")) {
                    matches++;
                }
            }
            assertEquals(r.id() + ": '" + r.title() + "'", 1, matches);
        }
    }

    /**
     * Situation: the family file keeps its own assumption. Expected: every route says SELF, and the start tile that every route names is the tile of its start pose.
     */
    @Test
    public void everyRouteAssumesWeShootTheFirstTipAndNamesItsStartTile() {
        assertEquals(java.util.Collections.<String>emptyList(), RouteCatalog.familyProblems("SelfTipRoutes", routes(), FirstTip.SELF));
        for (AutoRoute r : routes()) {
            assertEquals(r.id(), FirstTip.SELF, r.firstTip());
            assertEquals(r.id(), org.firstinspires.ftc.teamcode.autoshoot.core.TileGrid.nameAt(r.startPose().x, r.startPose().y), r.startTile());
            assertEquals(r.id(), Alliance.RED, r.alliance());
        }
    }

    /**
     * Situation: the text a first-year student reads. Expected: every description names the points, the spare POLLEN, what can go wrong and what the partner may do,
     *     and every route has a note on its start that says WHY that start was chosen.
     */
    @Test
    public void everyDescriptionSaysThePointsTheSpareWhatCanGoWrongAndThePartner() {
        for (AutoRoute r : routes()) {
            String d = r.description();
            for (String part : new String[] {"POINTS", "SPARE POLLEN", "WHAT CAN GO WRONG", "THE PARTNER"}) {
                assertTrue(r.id() + " says " + part, d.contains(part));
            }
            assertTrue(r.id() + " has a start note that says why", r.step(0).note().contains("shooting pose"));
        }
    }

    /**
     * Situation: every route set down by hand 1 in and 3 degrees off, for both alliances. Expected: still legal and still valid (the placement margin).
     */
    @Test
    public void everyStartKeepsThePlacementMarginForBothAlliances() {
        for (AutoRoute red : routes()) {
            for (Alliance a : Alliance.values()) {
                StartRobustness.Report r = StartRobustness.check(red.forAlliance(a));
                assertTrue(red.id() + " " + a + ": " + r.summary(), r.placementSafe());
            }
        }
    }

    /**
     * Situation: the points each route claims, worked out by hand. LEAVE 3 + PARK 5 + one HIVE tip 20 (our own first volley: 3 NECTAR + 3 POLLEN) = 28 for a route that
     *     parks in the LOADING ZONE off the wall; 23 for one that only steps off (LEAVE + tip). A route never claims a second tip.
     */
    @Test
    public void thePointsAreWhatTheRouteRulesGive() {
        int leave = BioBuzzConstants.Auto.LEAVE_POINTS;
        int park = BioBuzzConstants.Auto.PARK_POINTS;
        int tip = BioBuzzConstants.Auto.TIP_POINTS;
        assertEquals(28, leave + park + tip);
        for (String id : new String[] {SelfTipRoutes.ANCHOR_SHOOT_PARK, SelfTipRoutes.GARDEN_CARRY_PARK, SelfTipRoutes.GARDEN_TOPUP_PARK}) {
            assertEquals(id, leave + park + tip, route(id).claimablePoints());
        }
        for (String id : new String[] {SelfTipRoutes.SHOOT_STEP_OFF, SelfTipRoutes.GARDEN_REFILL_READY, SelfTipRoutes.GARDEN_TOPUP_STEP_OFF, SelfTipRoutes.REAR_BANK}) {
            assertEquals(id, leave + tip, route(id).claimablePoints());
        }
        assertEquals("no LEAVE (the robot ends on the wall), no PARK: the tip only", tip, route(SelfTipRoutes.SHOOT_ONLY).claimablePoints());
        for (AutoRoute r : routes()) {
            RouteAnalysis a = r.analyze();
            assertEquals(r.id() + " tips the HIVE once, with its own first volley", 1, a.tips());
            assertEquals(r.id() + " claims no tip of the partner", 0, a.partnerTipPoints());
            assertEquals(r.id(), a.leave(), a.endWallGapIn() >= BioBuzzConstants.Auto.LEAVE_WALL_MARGIN_IN);
        }
    }

    /**
     * Situation: the first volley of every route (the first tip). Expected: it is the first SHOOT, fires 4, tips the HIVE with 3 POLLEN (4 pre-loads + 3 NECTAR: FIRST's
     *     calibration) and is fired from the start pose itself, with no drive before it, because every start is a shooting pose.
     */
    @Test
    public void theFirstVolleyIsFiredFromTheStartPoseAndTipsTheHiveWithThreePollen() {
        for (AutoRoute r : routes()) {
            RouteAnalysis a = r.analyze();
            RouteAnalysis.StepInfo first = a.step(1);
            assertEquals(r.id(), StepKind.SHOOT, first.step.kind());
            assertEquals(r.id(), 4, ((Step.Shoot) first.step).count());
            assertTrue(r.id() + " tips", first.tipped);
            assertEquals(r.id() + " 3 POLLEN tip a cell with 3 NECTAR", 3, first.tipNeeded);
            assertEquals(r.id(), 0.0, r.startPose().distanceTo(first.shootPose), 1e-9);
            assertEquals(r.id() + " fires at once", 0.0, first.startSec, 1e-9);
        }
    }

    /**
     * Situation: every volley of every route. Expected: the robot stands within the TeleOp planner's cone of {@code AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG} around the
     *     direction the cell faces, measured at the aim point. The route validator checks the standoff band and the aim of the back, but NOT this cone, and a shot
     *     from the side of a cell sees its opening edge-on (the first idea of this file, a shot at the rear cell from the LOADING ZONE, was 84 degrees off).
     */
    @Test
    public void everyVolleyIsInsideTheShootingCone() {
        for (AutoRoute r : routes()) {
            RouteAnalysis a = r.analyze();
            for (RouteAnalysis.StepInfo info : a.steps()) {
                if (!(info.step instanceof Step.Shoot)) {
                    continue;
                }
                CellSpec cell = a.field().cell(info.targetSide);
                Point2 launch = ShotGeometry.launchPoint(info.shootPose);
                double dx = launch.x - cell.originX;
                double dy = launch.y - cell.originY;
                double along = dx * Math.cos(cell.facingHeading) + dy * Math.sin(cell.facingHeading);
                double azimuth = Math.toDegrees(Math.acos(along / Math.hypot(dx, dy)));
                assertTrue(r.id() + " step " + (info.index + 1) + " stands " + azimuth + " degrees off the cell's direction",
                        azimuth <= BioBuzzConstants.AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG);
                double standoff = ShotGeometry.standoff(info.shootPose, cell);
                assertTrue(r.id() + " step " + (info.index + 1) + " is " + standoff + " in from the aim point",
                        standoff >= BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN && standoff <= BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN);
            }
        }
    }

    /**
     * Situation: the top-up routes. Expected: the second volley is a SHOOT at the AUDIENCE cell that the plan thinks is already DOWN, and it carries the one G417 WARN
     *     of that kind, with a note that says the camera decides; the first volley's tip is the only tip, so the plan never claims a second 20.
     */
    @Test
    public void theTopUpVolleyIsGatedByTheCameraAndNeverClaimsAnotherTip() {
        for (String id : new String[] {SelfTipRoutes.GARDEN_TOPUP_PARK, SelfTipRoutes.GARDEN_TOPUP_STEP_OFF}) {
            AutoRoute r = route(id);
            int topUpIndex = -1;
            for (int i = 2; i < r.size(); i++) {
                if (r.step(i).kind() == StepKind.SHOOT) {
                    topUpIndex = i;
                }
            }
            assertTrue(id + " has a second volley", topUpIndex > 0);
            RouteAnalysis.StepInfo info = r.analyze().step(topUpIndex);
            assertFalse(id + ": the plan sees the audience cell as DOWN", info.targetWasUp);
            assertTrue(id + ": the note says the camera decides", r.step(topUpIndex).note().contains("camera"));
            int count = 0;
            for (Issue i : warnings(r, Rule.SHOOT_TARGET)) {
                count += i.stepIndex == topUpIndex ? 1 : 0;
            }
            assertEquals(id, 1, count);
            assertEquals(id, 1, r.analyze().tips());
        }
        // the routes that do not top up have no volley after the first, except the bank at the rear cell
        for (String id : new String[] {SelfTipRoutes.ANCHOR_SHOOT_PARK, SelfTipRoutes.SHOOT_STEP_OFF, SelfTipRoutes.SHOOT_ONLY, SelfTipRoutes.GARDEN_CARRY_PARK,
                SelfTipRoutes.GARDEN_REFILL_READY}) {
            int volleys = 0;
            for (int i = 0; i < route(id).size(); i++) {
                volleys += route(id).step(i).kind() == StepKind.SHOOT ? 1 : 0;
            }
            assertEquals(id, 1, volleys);
        }
    }

    /**
     * Situation: the bank at the rear cell. Expected: the volley is at the cell that came up after our own tip, so it depends on the camera (no camera, no shot), it
     *     fires 4 into an EMPTY cell (4 of the 8 the second tip needs: no tip) and the route still keeps its LEAVE if the volley is skipped.
     */
    @Test
    public void theRearBankIsFiredOnlyAfterTheCameraSeesTheCellUpAndStillKeepsTheLeaveWithoutIt() {
        AutoRoute r = route(SelfTipRoutes.REAR_BANK);
        RouteAnalysis a = r.analyze();
        assertTrue(a.dependsOnTipConfirmation());
        RouteAnalysis.StepInfo bank = a.step(r.size() - 1);
        assertEquals(StepKind.SHOOT, bank.step.kind());
        assertTrue("it is the cell that came up", bank.targetWasUp && bank.upSideBefore == bank.targetSide);
        assertFalse("4 POLLEN do not tip an empty cell", bank.tipped);
        assertEquals(0, bank.nectarInCellBefore);
        assertEquals(4, bank.pollenInCellAfter);
        RouteAnalysis without = a.giveUpPlan();
        assertNotNull(without);
        assertEquals("the LEAVE does not depend on the bank", a.leave(), without.leave());
        assertEquals(a.points(), without.points());
    }

    /**
     * Situation: the seconds the route texts quote. The anchor start is faster than tile C1 and B1 with a drive to the shot (the text of the basic route says 0.7 s and
     *     1.5 s); the turned start saves about 1.0 s on the top-up route and about 0.4 s on a route with one trip (the note of the top-up route says both); the route that
     *     steps off is about 3 s shorter than the one that parks (the text says 3 s).
     */
    @Test
    public void theSecondsThatTheTextsQuoteAreTheSecondsTheCodeComputes() {
        double anchor = route(SelfTipRoutes.ANCHOR_SHOOT_PARK).estimatedSeconds();
        // the same plan from tile C1 and B1 (front on the audience wall), the solver choosing the shooting pose: what the old autos' starts cost
        double c1 = fromTile(2).estimatedSeconds();
        double b1 = fromTile(1).estimatedSeconds();
        assertEquals("the basic route says 0.7 s against tile C1", 0.7, c1 - anchor, 0.15);
        assertEquals("the basic route says 1.5 s against tile B1", 1.5, b1 - anchor, 0.2);
        assertTrue(route(SelfTipRoutes.ANCHOR_SHOOT_PARK).description().contains("0.7 s against tile C1"));
        assertTrue(route(SelfTipRoutes.ANCHOR_SHOOT_PARK).description().contains("1.5 s against tile B1"));
        // the turned start on the top-up route that makes two trips to the GARDEN
        double turnedTopUp = SelfTipRoutes.gardenTopUpPark(SelfTipRoutes.TURNED_START).estimatedSeconds();
        double anchorTopUp = SelfTipRoutes.gardenTopUpPark(SelfTipRoutes.ANCHOR_START).estimatedSeconds();
        assertEquals("the top-up note says about 1.0 s", 1.0, anchorTopUp - turnedTopUp, 0.3);
        assertTrue(route(SelfTipRoutes.GARDEN_TOPUP_PARK).step(0).note().contains("about 1.0 s"));
        // and on a route with one trip along the wall it saves only 0.4 s (the same note)
        double oneTrip = fromStart(SelfTipRoutes.ANCHOR_START).estimatedSeconds() - fromStart(SelfTipRoutes.TURNED_START).estimatedSeconds();
        assertEquals("the note says 0.4 s", 0.4, oneTrip, 0.15);
        assertTrue(route(SelfTipRoutes.GARDEN_TOPUP_PARK).step(0).note().contains("just 0.4 s"));
        // the park costs about 3 s: the top-up route that steps off is that much shorter than the one that parks
        double withPark = route(SelfTipRoutes.GARDEN_TOPUP_PARK).estimatedSeconds();
        double stepOff = route(SelfTipRoutes.GARDEN_TOPUP_STEP_OFF).estimatedSeconds();
        assertTrue("the step-off route is the shorter: " + stepOff + " against " + withPark, stepOff < withPark);
        assertTrue(route(SelfTipRoutes.GARDEN_TOPUP_STEP_OFF).description().contains("3 s earlier"));
        assertEquals(3.0, withPark - stepOff, 0.5);
    }

    /** The basic route's legs from a start that is a shooting pose (fired from the start itself), for the seconds the note quotes. */
    private static AutoRoute fromStart(Pose2 s) {
        return AutoRoute.builder("park").title("park").startTile("B1").start(s.x, s.y, s.headingDegrees())
                .shootUp(s.x, s.y, s.headingDegrees(), 4)
                .driveConstant(30.0, 14.8, -90.0)
                .driveReverseTangent(30.0, 84.0)
                .driveLinear(SelfTipRoutes.PARK_LOW.x, SelfTipRoutes.PARK_LOW.y, 0.0)
                .park(SelfTipRoutes.PARK_LOW.x, SelfTipRoutes.PARK_LOW.y, 0.0).build();
    }

    /** The basic route's legs from a standard tile start (front on the audience wall, the solver picks the shooting pose), for the seconds the text quotes. */
    private static AutoRoute fromTile(int tileColumn) {
        double x = org.firstinspires.ftc.teamcode.autoshoot.core.TileGrid.centerX(tileColumn);
        return AutoRoute.builder("tile").title("tile").startTile("B1").start(x, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90.0)
                .shootUp(4)
                .driveConstant(30.0, 14.8, -90.0)
                .driveReverseTangent(30.0, 84.0)
                .driveLinear(SelfTipRoutes.PARK_LOW.x, SelfTipRoutes.PARK_LOW.y, 0.0)
                .park(SelfTipRoutes.PARK_LOW.x, SelfTipRoutes.PARK_LOW.y, 0.0).build();
    }

    /**
     * Situation: no two routes are the same plan. Expected: no two routes share both the same kinds of steps and the same end pose, and any two routes that do
     *     start at the same pose end differently or take a different time of at least 1 s.
     */
    @Test
    public void noTwoRoutesAreNearDuplicates() {
        List<AutoRoute> all = routes();
        for (int i = 0; i < all.size(); i++) {
            for (int j = i + 1; j < all.size(); j++) {
                AutoRoute a = all.get(i);
                AutoRoute b = all.get(j);
                List<StepKind> ka = new ArrayList<StepKind>();
                List<StepKind> kb = new ArrayList<StepKind>();
                for (Step s : a.steps()) {
                    ka.add(s.kind());
                }
                for (Step s : b.steps()) {
                    kb.add(s.kind());
                }
                boolean sameStart = a.startPose().distanceTo(b.startPose()) < 0.5;
                boolean sameEnd = a.endPoses().get(a.size() - 1).distanceTo(b.endPoses().get(b.size() - 1)) < 0.5;
                boolean sameKinds = ka.equals(kb);
                String pair = a.id() + " and " + b.id();
                assertFalse(pair + " have the same steps and the same end", sameKinds && sameEnd);
                if (sameStart && sameEnd) {
                    assertTrue(pair + " start and end alike must differ in what they do", !sameKinds);
                }
                if (sameKinds && sameStart) {
                    assertTrue(pair + " differ by at least 1 s", Math.abs(a.estimatedSeconds() - b.estimatedSeconds()) >= 1.0 || !sameEnd);
                }
            }
        }
    }

    /**
     * Situation: the catalog's routes named in the file. Expected: the constants are the ids in the list, in the order the team reads them.
     */
    @Test
    public void theIdConstantsAreTheRoutesInOrder() {
        List<String> ids = new ArrayList<String>();
        for (AutoRoute r : routes()) {
            ids.add(r.id());
        }
        assertEquals(Arrays.asList(SelfTipRoutes.ANCHOR_SHOOT_PARK, SelfTipRoutes.SHOOT_STEP_OFF, SelfTipRoutes.SHOOT_ONLY, SelfTipRoutes.GARDEN_CARRY_PARK,
                SelfTipRoutes.GARDEN_REFILL_READY, SelfTipRoutes.GARDEN_TOPUP_STEP_OFF, SelfTipRoutes.GARDEN_TOPUP_PARK, SelfTipRoutes.REAR_BANK), ids);
    }

    /**
     * Situation: the poses of the file. Expected: the anchor start is the anchor shot spot (front edge on the audience wall, in line with the HIVE); the parking pose
     *     overlaps the LOADING ZONE by 6.5 in and stands 4 in off the red wall (the texts say so); the rear shooting pose is clear of the rear FLOWER's plate.
     */
    @Test
    public void theSharedPosesAreWhatTheTextsSay() {
        Pose2 anchor = SelfTipRoutes.ANCHOR_START;
        assertEquals(BioBuzzConstants.Robot.FRONT_EXTENT_IN, anchor.y, 1e-9);
        assertEquals(BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, anchor.x, 1e-9);
        Pose2 park = SelfTipRoutes.PARK_LOW;
        double overlapY = park.y + BioBuzzConstants.Robot.LEFT_EXTENT_IN - BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN;
        assertEquals(6.5, overlapY, 1e-9);
        assertEquals("4 in off the red wall", 4.0, park.x - BioBuzzConstants.Robot.REAR_EXTENT_IN, 0.05);
        Pose2 rear = SelfTipRoutes.REAR_SHOT;
        double plateRight = BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][0] + BioBuzzConstants.Field.FLOWER_PLATE_ALONG_WALL_IN / 2.0;
        double robotLeft = rear.x - BioBuzzConstants.Robot.LEFT_EXTENT_IN;
        assertEquals("the note says " + SelfTipRoutes.rearPlateGapText() + " in clear of the rear FLOWER's plate", Double.parseDouble(SelfTipRoutes.rearPlateGapText()),
                robotLeft - plateRight, 0.05);
        boolean said = false;
        for (AutoRoute r : routes()) {
            if (r.id().equals(SelfTipRoutes.REAR_BANK)) {
                for (Step s : r.steps()) {
                    said |= s.note().contains(SelfTipRoutes.rearPlateGapText() + " in clear of the rear-wall FLOWER's plate");
                }
            }
        }
        assertTrue("the note of the move says it", said);
        assertEquals("the text says 3.6 in off the rear wall", 3.6,
                BioBuzzConstants.Field.SIZE_IN - (rear.y + BioBuzzConstants.Robot.FRONT_EXTENT_IN), 0.05);
    }
}
