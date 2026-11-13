package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.regex.Pattern;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The partner-tips-first routes (PartnerTipRoutes): the alliance partner shoots the first tip of our HIVE, our robot goes to the REAR cell that comes up, waits
 * until its camera has SEEN that cell up and steady, shoots 4 into it, then leaves and parks. These tests hold every route of the family to the same list:
 * valid for both alliances, placeable with the 1 in and 3 degree margin, a WAIT_FOR_TIP before every shot at the cell that comes up, a slack for a late partner,
 * a title that starts with a tier word, and different from the others. The numbers the texts quote are checked against what the code computes.
 */
public class PartnerTipRoutesTest {
    private static final double EPS = 1e-9;
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final Pattern TITLE = Pattern.compile("^(Safe|Standard|Aggressive|Advanced): \\S.*");
    private static final Pattern ID = Pattern.compile("^[a-z0-9]+(_[a-z0-9]+)*$");

    /** The routes written by the "start somewhere else" part of the file (everything but the single corner route that was there first). */
    private static List<AutoRoute> newRoutes() {
        List<AutoRoute> out = new ArrayList<AutoRoute>();
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            if (!r.id().equals(PartnerTipRoutes.PARTNER_REAR_SHOOT_PARK)) {
                out.add(r);
            }
        }
        return out;
    }

    private static AutoRoute byId(String id) {
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            if (r.id().equals(id)) {
                return r;
            }
        }
        throw new AssertionError("no partner route " + id);
    }

    private static CellSpec rearCell() {
        return new AllianceField(new FieldModel(), Alliance.RED).cell(Side.REAR);
    }

    private static RouteAnalysis.StepInfo firstOf(RouteAnalysis a, Class<? extends Step> kind) {
        for (RouteAnalysis.StepInfo s : a.steps()) {
            if (kind.isInstance(s.step)) {
                return s;
            }
        }
        throw new AssertionError("no " + kind.getSimpleName());
    }

    // ---- the list -------------------------------------------------------------------------------------------------------------------------

    /**
     * Situation: the file as a whole.
     * Expected: at least 8 routes besides the corner route, every one marked partnerTipsFirst(), ids unique (also across the whole catalog), lowercase with underscores.
     */
    @Test
    public void theFamilyHasEightNewRoutesAllPartnerFirstWithUniqueLowercaseIds() {
        List<AutoRoute> routes = PartnerTipRoutes.routes();
        assertTrue("at least 8 new routes: " + newRoutes().size(), newRoutes().size() >= 8);
        Set<String> ids = new HashSet<String>();
        for (AutoRoute r : routes) {
            assertEquals(r.id() + " assumes the partner shoots the first tip", FirstTip.PARTNER, r.firstTip());
            assertTrue(r.id() + " is lowercase with underscores", ID.matcher(r.id()).matches());
            assertTrue("duplicate id " + r.id(), ids.add(r.id()));
        }
        Set<String> catalog = new HashSet<String>();
        for (AutoRoute r : RouteCatalog.all()) {
            assertTrue("duplicate id in the catalog: " + r.id(), catalog.add(r.id()));
        }
        assertTrue(catalog.containsAll(ids));
    }

    /** Situation: every title. Expected: it starts with exactly one tier word and a colon, and the four tier words are the only ones. */
    @Test
    public void everyTitleStartsWithOneTierWordAndAColon() {
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            assertTrue(r.id() + ": '" + r.title() + "'", TITLE.matcher(r.title()).matches());
            String rest = r.title().substring(r.title().indexOf(':') + 1);
            for (String tier : new String[]{"Safe:", "Standard:", "Aggressive:", "Advanced:"}) {
                assertFalse(r.id() + " has a second tier word in its title", rest.contains(tier));
            }
        }
    }

    /**
     * Situation: every route, for RED and for BLUE (the half turn).
     * Expected: no ERROR. A WARN is allowed only if the step it belongs to carries a note that explains it; today there is none at all.
     */
    @Test
    public void everyRouteValidatesWithNoErrorForBothAlliances() {
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            for (Alliance a : Alliance.values()) {
                for (Issue i : RouteValidator.check(r.forAlliance(a))) {
                    assertFalse(r.id() + " " + a + ": " + i.message, i.severity == Severity.ERROR);
                    assertTrue(r.id() + " " + a + " WARN on a step without a note: " + i.message, !r.step(i.stepIndex).note().isEmpty());
                }
            }
        }
    }

    /**
     * Situation: a hand sets the robot down 1 in and 3 degrees off, in every combination the placement check tries.
     * Expected: every placement is still legal and still valid for every route and both alliances (the route survives the placement margin).
     */
    @Test
    public void everyRouteSurvivesThePlacementMargin() {
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            for (Alliance a : Alliance.values()) {
                StartRobustness.Report report = StartRobustness.check(r.forAlliance(a));
                assertTrue(r.id() + " " + a + ": " + report.summary() + " " + (report.newErrors().isEmpty() ? "" : report.newErrors().get(0).message), report.placementSafe());
            }
        }
    }

    // ---- the camera decides ---------------------------------------------------------------------------------------------------------------

    /**
     * Situation: every SHOOT at the cell that comes up with the partner's tip.
     * Expected: a WAIT_FOR_TIP stands before it, ends at the tip plus the settle time at the earliest (the camera has seen the cell steady), and the volley starts when
     *     it ends, so the robot never fires into a cell that is still down and never because a time has passed. The give-up time is the latest the rest of the route allows.
     */
    @Test
    public void everyShotAtTheCellThatComesUpWaitsForTheCameraFirst() {
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            RouteAnalysis a = r.analyze();
            assertTrue(r.id(), a.partnerTipAssumed());
            Side comesUp = a.field().initialUpSide().opposite();
            int shoots = 0;
            for (RouteAnalysis.StepInfo s : a.steps()) {
                if (!(s.step instanceof Step.Shoot)) {
                    continue;
                }
                shoots++;
                assertEquals(r.id() + " shoots at the cell that comes up", comesUp, s.targetSide);
                assertTrue(r.id() + " the tip is there before the volley", s.targetWasUp);
                int wait = -1;
                for (int i = 0; i < s.index; i++) {
                    if (r.step(i).kind() == StepKind.WAIT_FOR_TIP) {
                        wait = i;
                    }
                }
                assertTrue(r.id() + " has a WAIT_FOR_TIP before the shot", wait >= 0);
                assertTrue(r.id() + " the wait lasts until the tip is confirmed (the tip plus the settle time)", a.step(wait).endSec() >= a.partnerTipConfirmedSec() - EPS);
                assertTrue(r.id() + " the volley starts only after that", s.launchSec >= a.partnerTipConfirmedSec() - EPS);
                assertTrue(r.id() + " is a tip-dependent volley", s.tipDependent);
            }
            assertEquals(r.id() + " fires one volley", 1, shoots);
        }
    }

    /**
     * Situation: the give-up time of every wait.
     * Expected: it is the latest the rest of the route allows, rounded down to a tenth of a second (not later: the route would overrun the AUTO limit; not more than
     *     0.1 s earlier: a partner that is slow but fine would be turned away), and the text of the route quotes it.
     */
    @Test
    public void theGiveUpTimeIsTheLatestTheRouteAllowsAndTheTextQuotesIt() {
        for (AutoRoute r : newRoutes()) {
            RouteAnalysis a = r.analyze();
            RouteAnalysis.StepInfo wait = firstOf(a, Step.WaitForTip.class);
            double giveUp = ((Step.WaitForTip) wait.step).giveUpSec();
            double latest = a.latestGiveUpSec(wait);
            assertTrue(r.id() + ": " + giveUp + " is later than " + latest, giveUp <= latest + EPS);
            assertTrue(r.id() + ": " + giveUp + " is more than 0.1 s earlier than " + latest, giveUp > latest - 0.1 - EPS);
            assertTrue(r.id() + " says the give-up time", r.description().contains(String.format(Locale.ROOT, "gives up waiting at %.1f s", giveUp)));
        }
    }

    // ---- how late may the partner be -----------------------------------------------------------------------------------------------------

    /**
     * Situation: the slack of every partner route.
     * Expected: the latest partner tip that still works is finite and later than the assumed one, the slack is positive and at least the minimum the validator asks for
     *     (no WARN), and the text of the route prints both numbers, so the drive team can pick the route that matches the partner's auto.
     */
    @Test
    public void everyRouteReportsAPositiveSlackForTheAssumedPartnerTime() {
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            RouteAnalysis a = r.analyze();
            double latest = a.latestPartnerTipSec();
            double slack = a.partnerTipSlackSec();
            assertFalse(r.id() + " has a latest partner tip", Double.isNaN(latest) || Double.isInfinite(latest));
            assertEquals(r.id() + ": the slack is the latest tip minus the assumed one", latest - BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC, slack, EPS);
            assertTrue(r.id() + " slack " + slack, slack >= BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC);
        }
        for (AutoRoute r : newRoutes()) {
            RouteAnalysis a = r.analyze();
            assertTrue(r.id() + " prints the latest partner tip", r.description().contains(String.format(Locale.ROOT, "tips as late as %.1f s", a.latestPartnerTipSec())));
            assertTrue(r.id() + " prints the slack", r.description().contains(String.format(Locale.ROOT, "which is %.1f s of slack", a.partnerTipSlackSec())));
        }
    }

    /**
     * Situation: the route played with the partner's tip at the latest time it reports, and a little later.
     * Expected: at the latest time the robot still shoots and the route still ends inside the budget; 0.5 s later the wait has given up and the volley is not fired.
     */
    @Test
    public void theLatestPartnerTipReallyIsTheLatest() {
        for (AutoRoute r : newRoutes()) {
            double latest = r.analyze().latestPartnerTipSec();
            RouteAnalysis ok = RouteAnalysis.ofWithPartnerTipAt(r, latest);
            assertTrue(r.id() + " ends at " + ok.totalSeconds(), ok.totalSeconds() <= RouteAnalysis.timeBudgetSec() + 0.05);
            RouteAnalysis.StepInfo shoot = firstOf(ok, Step.Shoot.class);
            assertTrue(r.id() + " still shoots after the tip", shoot.launchSec >= latest + ok.tipSettleSec() - EPS);
            double lateTip = latest + 0.5;
            RouteAnalysis tooLate = RouteAnalysis.ofWithPartnerTipAt(r, lateTip);
            assertTrue(r.id() + ": a tip 0.5 s later is confirmed after the wait has given up", lateTip + tooLate.tipSettleSec() > ((Step.WaitForTip) firstOf(tooLate, Step.WaitForTip.class).step).giveUpSec());
        }
    }

    // ---- points and timing ---------------------------------------------------------------------------------------------------------------

    /**
     * Situation: what each new route can claim.
     * Expected, worked out by hand: LEAVE 3 + PARK 5 = 8 points that are ours; our 4 POLLEN go into the cell that came up EMPTY, which needs 8, so nothing of ours
     *     tips (no 20 of ours) and the cell holds 4; the partner's 20 is in the plan but never in our points; every POLLEN is shot (the robot ends with 0).
     */
    @Test
    public void everyNewRouteIsWorthEightPointsAndBanksFourPollen() {
        int leave = BioBuzzConstants.Auto.LEAVE_POINTS;
        int park = BioBuzzConstants.Auto.PARK_POINTS;
        assertEquals(8, leave + park);
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            for (Alliance al : Alliance.values()) {
                RouteAnalysis a = r.forAlliance(al).analyze();
                assertTrue(r.id() + " " + al + " keeps its LEAVE", a.leave());
                assertTrue(r.id() + " " + al + " earns its PARK", a.park());
                assertEquals(r.id() + " " + al, leave + park, a.points());
                assertEquals(r.id() + " none of our volleys tips", 0, a.tips());
                assertEquals(r.id() + " the partner's tip is planned", 1, a.partnerTips());
                assertEquals(r.id() + " and is never counted as ours", BioBuzzConstants.Auto.TIP_POINTS, a.partnerTipPoints());
                RouteAnalysis.StepInfo shoot = firstOf(a, Step.Shoot.class);
                assertEquals(r.id() + " the empty cell holds our 4", 4, shoot.pollenInCellAfter);
                assertEquals(r.id() + " the cell that came up holds no NECTAR", 0, shoot.nectarInCellBefore);
                assertEquals(r.id() + " ends with no POLLEN held", 0, a.pollenEnd());
            }
        }
    }

    /**
     * Situation: the clock of a route, worked out by hand for the volley.
     * Expected: the partner tips at 12 s, the camera confirms 0.5 s later (12.5 s), the volley starts then and lasts the aim and spin-up plus 4 feeds, and every route
     *     ends inside the AUTO budget with at least the minimum slack to spare.
     */
    @Test
    public void theVolleyStartsWhenTheCameraConfirmsAndEveryRouteFinishesWithMargin() {
        double confirmed = BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC + BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
        double volley = BioBuzzConstants.Auto.AIM_SPINUP_SEC + 4 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC;
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            RouteAnalysis a = r.analyze();
            RouteAnalysis.StepInfo shoot = firstOf(a, Step.Shoot.class);
            if (newRoutes().contains(r)) {
                // (the corner route steps 1 in toward the cell first, so its volley starts a moment after the confirmation)
                assertEquals(r.id() + " fires when the tip is confirmed", confirmed, shoot.launchSec, 1e-6);
                assertEquals(r.id() + " the volley's length", volley, shoot.durationSec, 1e-6);
            }
            assertTrue(r.id() + " ends at " + a.totalSeconds(), a.totalSeconds() <= RouteAnalysis.timeBudgetSec() - BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC);
        }
    }

    /**
     * Situation: where the robot is while the partner shoots.
     * Expected: it is at its shooting pose before the partner's assumed tip, so the 12 s wait hides the drive (the text says so), and the route that starts nearest the
     *     cell is the one that is ready first.
     */
    @Test
    public void everyNewRouteIsReadyBeforeThePartnerTipsAndTheAggressiveOneIsReadyFirst() {
        double fastest = Double.POSITIVE_INFINITY;
        String fastestId = "";
        for (AutoRoute r : newRoutes()) {
            double ready = firstOf(r.analyze(), Step.WaitForTip.class).startSec;
            assertTrue(r.id() + " ready at " + ready, ready < BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC - 3.0);
            assertTrue(r.id() + " says when it is ready", r.description().contains(String.format(Locale.ROOT, "at its shooting pose %.1f s after AUTO starts", ready)));
            if (ready < fastest) {
                fastest = ready;
                fastestId = r.id();
            }
        }
        assertEquals(PartnerTipRoutes.AGGRESSIVE_REAR_EDGE, fastestId);
    }

    // ---- the shot ---------------------------------------------------------------------------------------------------------------------------

    /**
     * Situation: where each new route shoots from.
     * Expected: inside the 30 to 38 in band, with the back at the cell, and inside the cone the TeleOp solver uses (20 degrees either side of the direction the cell
     *     faces: a shot from farther round sees a narrower window in the opening). The one old route of the file (PARTNER_REAR_SHOOT_PARK, turned 35 degrees in the
     *     corner) is not held to the cone: its shot is about 38 degrees off the axis, which the validator does not check but the solver would not choose.
     */
    @Test
    public void everyNewRouteShootsInsideTheBandAndTheConeOfTheSolver() {
        CellSpec cell = rearCell();
        for (AutoRoute r : newRoutes()) {
            Pose2 p = firstOf(r.analyze(), Step.Shoot.class).shootPose;
            double standoff = ShotGeometry.standoff(p, cell);
            assertTrue(r.id() + " standoff " + standoff, standoff >= BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN && standoff <= BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN);
            assertTrue(r.id() + " back at the cell", Math.toDegrees(ShotGeometry.aimError(p, cell)) <= BioBuzzConstants.Auto.SHOOT_AIM_TOLERANCE_DEG);
            Point2 launch = ShotGeometry.launchPoint(p);
            double offAxis = Math.toDegrees(Math.atan2(Math.abs(launch.x - cell.originX), launch.y - cell.originY));
            assertTrue(r.id() + " is " + offAxis + " degrees off the cell's axis", offAxis <= BioBuzzConstants.AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG);
        }
    }

    /**
     * Situation: the numbers the texts and notes quote for the three shooting poses and the two park poses, computed by hand here from the field constants.
     * Expected: pose 1 (60.5, 130.98): launch point 5.38 in behind the center, 38.5 in from the aim point, left edge 0.9 in past the FLOWER keep-out, right edge
     *     1.4 in short of the center line; pose 2 on the HIVE axis 32.8 in out; pose 3 33.0 in out, 10 degrees to the left of the axis, 1.4 in clear of the plate.
     *     The route texts say the same numbers (they are worked out by the code), and the shots are inside the band of the validator.
     */
    @Test
    public void theQuotedShootingPoseNumbersAreRight() {
        CellSpec cell = rearCell();
        Rect plate = new AllianceField(new FieldModel(), Alliance.RED).flowerPlate(1);
        double keepOutRight = plate.maxX + BioBuzzConstants.Auto.FLOWER_KEEPOUT_MARGIN_IN;
        // pose 1
        Pose2 one = firstOf(byId(PartnerTipRoutes.SAFE_REAR_WALL_B6).analyze(), Step.Shoot.class).shootPose;
        assertEquals(60.5, one.x, 1e-9);
        // facing +y the robot's left is -x, so the ball leaves the sideways offset (negative: right of the middle) toward +x
        double dx = (one.x - BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN) - cell.originX;
        double dy = (one.y + BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN) - cell.originY;
        assertEquals("by hand: the hypotenuse of the launch point to the aim point", Math.hypot(dx, dy), ShotGeometry.standoff(one, cell), 1e-9);
        assertEquals("38.5 in", 38.5, ShotGeometry.standoff(one, cell), 0.06);
        assertTrue("inside the band", ShotGeometry.standoff(one, cell) <= BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN);
        assertEquals("0.9 in past the FLOWER keep-out", 0.9, (one.x - BioBuzzConstants.Robot.LEFT_EXTENT_IN) - keepOutRight, 0.05);
        assertEquals("1.4 in short of the center line", 1.4, BioBuzzConstants.Field.CENTER_IN - (one.x + BioBuzzConstants.Robot.RIGHT_EXTENT_IN), 0.05);
        String b6 = byId(PartnerTipRoutes.SAFE_REAR_WALL_B6).description();
        assertTrue(b6, b6.contains(String.format(Locale.ROOT, "ends %.1f in from the FLOWER keep-out and %.1f in from the center line",
                PartnerTipRoutes.keepOutGap1(), PartnerTipRoutes.centerGap1())));
        assertEquals("0.6 in off the wall", 0.6, SIZE - BioBuzzConstants.Robot.FRONT_EXTENT_IN - one.y, 1e-9);
        // pose 2
        Pose2 two = firstOf(byId(PartnerTipRoutes.SAFE_RED_CORNER_A6).analyze(), Step.Shoot.class).shootPose;
        assertEquals("on the HIVE axis", BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, two.x, 1e-9);
        assertEquals("32.8 in", 32.8, ShotGeometry.standoff(two, cell), 0.05);
        assertTrue("the text says so: " + byId(PartnerTipRoutes.SAFE_RED_CORNER_A6).description(),
                byId(PartnerTipRoutes.SAFE_RED_CORNER_A6).description().contains("the shooting pose is 32.8 in from the aim point"));
        // pose 3
        Pose2 three = firstOf(byId(PartnerTipRoutes.AGGRESSIVE_REAR_EDGE).analyze(), Step.Shoot.class).shootPose;
        assertEquals("33.0 in", 33.0, ShotGeometry.standoff(three, cell), 0.06);
        Point2 launch = ShotGeometry.launchPoint(three);
        assertEquals("about 10 degrees left of the axis", 10.0, Math.toDegrees(Math.atan2(cell.originX - launch.x, launch.y - cell.originY)), 0.5);
        double clear = -1;
        for (double g = 0.0; g < 6.0; g += 0.05) {
            if (Footprint.of(three).overlaps(plate.inflate(g))) {
                break;
            }
            clear = g;
        }
        assertEquals("1.3 in from the FLOWER plate", 1.3, clear, 0.15);
    }

    /**
     * Situation: the two park poses.
     * Expected: both reach 7 in or more into the LOADING ZONE and stand clear of the red wall by more than the 1.5 in the LEAVE needs, as the notes say.
     */
    @Test
    public void theParkNotesQuoteTheRealOverlapAndWallGap() {
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            RouteAnalysis a = r.analyze();
            RouteAnalysis.StepInfo park = a.step(r.size() - 1);
            String note = park.step.note();
            assertTrue(r.id() + ": " + note, note.contains(String.format(Locale.ROOT, "%.1f in (the zone is 11 in deep)", a.parkOverlapIn())));
            double gap = Footprint.of(((Step.Park) park.step).pose()).bounds().minX;
            assertTrue(r.id() + ": " + note, note.contains(String.format(Locale.ROOT, "standing %.1f in off the wall", gap)));
            assertTrue(r.id() + " keeps LEAVE", gap >= BioBuzzConstants.Auto.LEAVE_WALL_MARGIN_IN);
        }
    }

    // ---- different from each other -------------------------------------------------------------------------------------------------------

    /**
     * Situation: all routes of the family side by side.
     * Expected: no two have the same start, and no two have the same list of poses; the starts cover the audience wall, the red wall and the rear wall; and every
     *     description is written (long enough to hold the five parts and the second-tip sentence).
     */
    @Test
    public void theRoutesAreDifferentFromEachOtherAndEachIsDescribed() {
        Set<String> starts = new HashSet<String>();
        Set<String> shapes = new HashSet<String>();
        Set<String> walls = new HashSet<String>();
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            Pose2 s = r.startPose();
            assertTrue(r.id() + " has the same start as another route", starts.add(String.format(Locale.ROOT, "%.1f/%.1f/%.0f", s.x, s.y, s.headingDegrees())));
            StringBuilder shape = new StringBuilder();
            for (RouteAnalysis.StepInfo info : r.analyze().steps()) {
                shape.append(String.format(Locale.ROOT, "%.1f,%.1f,%.0f;", info.to.x, info.to.y, info.to.headingDegrees()));
            }
            assertTrue(r.id() + " has the same poses as another route", shapes.add(shape.toString()));
            walls.add(s.x < BioBuzzConstants.Robot.LEFT_EXTENT_IN + 1.0 ? "red" : s.y < 20.0 ? "audience" : "rear");
        }
        assertTrue("the starts use every own wall: " + walls, walls.containsAll(java.util.Arrays.asList("audience", "red", "rear")));
        for (AutoRoute r : newRoutes()) {
            String d = r.description();
            for (String part : new String[]{"WHY THIS START", "READY", "THE PARTNER MUST", "WE DO", "OUR POINTS", "WHY NO SECOND TIP", "WHAT COULD GO WRONG"}) {
                assertTrue(r.id() + " is missing the part " + part, d.contains(part));
            }
            assertTrue(r.id() + " says that about 60 percent of shots land", d.contains("60 percent"));
            assertFalse(r.id() + " has the start tile", r.startTile().isEmpty());
        }
    }

    // ---- why there is no route that makes the second tip --------------------------------------------------------------------------------

    /**
     * Situation: the route a second tip would need: shoot 4 into the rear cell, fetch 4 from the rear FLOWER, shoot 4 again (8 in the cell), with the partner's tip at
     *     the assumed 12 s.
     * Expected: with the robot as shipped the validator refuses the pick-up (the flat wheels fall short of the POLLEN, an ERROR), and even with wheels that DO reach
     *     the plan ends after the AUTO budget and tolerates no partner tip at the assumed time (negative slack). That is why every route of the family banks 4 and
     *     takes LEAVE and PARK: the second tip fits only for a partner that tips about 6 s earlier than assumed, and 8 landed shots out of 8 is unlikely anyway.
     */
    @Test
    public void aSecondTipDoesNotFitSoNoRouteTriesIt() {
        double[] flower = BioBuzzConstants.Field.FLOWER_CENTERS_IN[1];
        AutoRoute second = AutoRoute.builder("second_tip_try").partnerTipsFirst().title("Aggressive: try the second tip")
                .start(28.0, SIZE - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90.0)
                .driveConstant(28.0, 123.0, 90.0)
                .driveConstant(60.5, 123.0, 90.0)
                .driveConstant(60.5, SIZE - BioBuzzConstants.Robot.FRONT_EXTENT_IN - 0.6, 90.0)
                .waitForTip(BioBuzzConstants.Auto.AUTO_DURATION_SEC)
                .shootUp(60.5, SIZE - BioBuzzConstants.Robot.FRONT_EXTENT_IN - 0.6, 90.0, 4)
                .driveConstant(60.5, 118.0, 90.0)
                .intakeFlower(flower[0], flower[1], 4)
                .shootUp(4)
                .park(12.0, 108.0, 0.0)
                .build();
        boolean wheelError = false;
        for (Issue i : RouteValidator.check(second)) {
            wheelError |= i.severity == Severity.ERROR && i.rule == Rule.FLOWER_PRESS;
        }
        assertTrue("the shipped wheels cannot reach the POLLEN, so the pick-up is an ERROR", wheelError);
        TimingModel reaching = StartShortcuts.wheelsThatReach();
        RouteAnalysis a = RouteAnalysis.of(second, new FieldModel(), reaching);
        assertNotNull(a.step(0));
        assertTrue("with wheels that reach it ends at " + a.totalSeconds() + " s, over the " + RouteAnalysis.timeBudgetSec() + " s budget",
                a.totalSeconds() > RouteAnalysis.timeBudgetSec());
        assertTrue("and tolerates no partner at 12 s: latest tip " + a.latestPartnerTipSec(), a.partnerTipSlackSec() < 0.0);
    }
}
