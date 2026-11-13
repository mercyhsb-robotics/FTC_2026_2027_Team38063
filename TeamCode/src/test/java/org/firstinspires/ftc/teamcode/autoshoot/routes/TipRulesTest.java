package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The validator rules about HIVE tips (G417 and G409 and the first-tip assumption), each with a route that breaks it
 * and a legal one next to it. Every route is validated for RED and for its BLUE half turn and both must agree.
 */
public class TipRulesTest {
    /** The robot's center of rotation is this far (9.77 in) from a wall its FRONT edge is touching; see RobotShape. */
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double TIP_SEC = BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC;
    /** The camera must see the tipped cell steady this long before the robot calls the HIVE fully tipped. */
    private static final double SETTLE = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
    private static final String CONFIRMED = String.format(java.util.Locale.ROOT, "%.1f s", TIP_SEC + SETTLE);

    /** Validates for RED and for its BLUE half turn; both must give the same findings. */
    private static List<Issue> validate(AutoRoute.Builder b) {
        AutoRoute red = b.build();
        List<Issue> r = RouteValidator.check(red);
        List<Issue> blue = RouteValidator.check(red.forAlliance(Alliance.BLUE));
        assertEquals("red and blue findings differ:\n" + r + "\n" + blue, signature(r), signature(blue));
        return r;
    }

    private static List<String> signature(List<Issue> issues) {
        List<String> out = new ArrayList<String>();
        for (Issue i : issues) {
            out.add(i.severity + "/" + i.rule + "/" + i.stepIndex);
        }
        return out;
    }

    private static int count(List<Issue> issues, Rule rule, Severity severity) {
        int n = 0;
        for (Issue i : issues) {
            if (i.rule == rule && i.severity == severity) {
                n++;
            }
        }
        return n;
    }

    private static Issue first(List<Issue> issues, Rule rule, Severity severity) {
        for (Issue i : issues) {
            if (i.rule == rule && i.severity == severity) {
                return i;
            }
        }
        throw new AssertionError("no " + severity + " for " + rule + " in " + issues);
    }

    /**
     * A partner route that starts on the rear wall (tile C6, legal start) and shoots at the REAR cell from a pose that
     * clears the rear-wall FLOWER: back toward the HIVE, 35 in from the aim point, 2.7 in from the start.
     */
    private static AutoRoute.Builder partnerRear() {
        return AutoRoute.builder("partner_rear").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst();
    }

    private static final double REAR_X = 61.5;
    private static final double REAR_Y = 130.0;
    /**
     * Where the partner routes drive to after their volley: off the rear wall. Every real route ends with such a safe step (LEAVE),
     * so that it is done whether or not the tip comes; without it the route that never gets its tip ends touching the wall.
     */
    private static final double OFF_WALL_Y = 112.0;

    /** The volley at the rear cell followed by the safe drive off the wall. */
    private static AutoRoute.Builder thenLeave(AutoRoute.Builder b) {
        return b.driveConstant(REAR_X, OFF_WALL_Y, 90);
    }

    // ---- the volley that tips: stops at the tip, spare, and the timed volley that cannot stop ---------------------

    @Test
    public void aCameraGatedVolleyThatIsBiggerThanTheTipNeedsIsFineForAFeederThatStopsInTime() {
        // 2 POLLEN, take 2 back from the floor, then fire 4: the 1st POLLEN of those 4 is the tip and a feeder that stops at once stops there
        AutoRoute.Builder b = AutoRoute.builder("late").start(58, FRONT, -90)
                .shootUp(2).intakeFloor(60, 30, BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, 30, -90, 2).shootUp(4);
        List<Issue> l = FastFeederRobot.check(b.build());
        assertEquals(l.toString(), 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        assertEquals(l.toString(), 0, count(l, Rule.SHOOT_TARGET, Severity.WARN));
        assertEquals("3 of the 4 are spare", 0, count(l, Rule.TIP_MARGIN, Severity.WARN));
    }

    @Test
    public void aTimedVolleyThatKeepsFiringAfterTheTipIsAWarning() {
        // an old-style volley that feeds for a fixed 6 s whatever happens: the 3rd of its 4 POLLEN tips the HIVE (3 NECTAR + 3
        // POLLEN) and the 4th would go into a cell that is already tipping down
        List<Issue> l = validate(AutoRoute.builder("timed").start(58, FRONT, -90).shootUp(4).launchSeconds(6.0));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.WARN));
        assertEquals("QA12 allows adding to a HIVE that moves, so a WARN and not an ERROR", 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.WARN);
        assertEquals("the volley is step 2", 1, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("TIMED"));
        assertTrue(issue.message, issue.message.contains("tips with POLLEN 3 of the 4"));
        assertTrue(issue.message, issue.message.contains("the last 1 would be launched into a cell that is already tipping down"));
        assertTrue(issue.message, issue.message.contains("QA12"));
        assertTrue(issue.message, issue.message.contains("G417"));
        assertTrue(issue.message, issue.message.contains("a cell tips at 8 POLLEN-equivalents"));
        // the same volley without the fixed time is gated by the camera: a feeder that stops in time has nothing to say...
        assertEquals(0, count(FastFeederRobot.check(AutoRoute.builder("gated").start(58, FRONT, -90).shootUp(4).build()), Rule.SHOOT_TARGET, Severity.WARN));
        // and a timed volley of exactly the 3 that tip fires nothing after the tip
        assertEquals(0, count(validate(AutoRoute.builder("exact").start(58, FRONT, -90).shootUp(3).launchSeconds(5.0)), Rule.SHOOT_TARGET,
                Severity.WARN));
    }

    @Test
    public void aVolleyWhoseLastPollenIsTheTipIsFine() {
        // 4 at once: the 3rd POLLEN tips and the 4th is the spare
        assertEquals(0, count(validate(AutoRoute.builder("one").start(58, FRONT, -90).shootUp(4)), Rule.SHOOT_TARGET, Severity.ERROR));
        // 2 + 1 into the same cell: the last POLLEN of the second volley tips
        assertEquals(0, count(validate(AutoRoute.builder("two").start(58, FRONT, -90).shootUp(2).shootUp(1)), Rule.SHOOT_TARGET,
                Severity.ERROR));
        // 2 POLLEN never reach the tip: nothing to complain about either
        assertEquals(0, count(validate(AutoRoute.builder("three").start(58, FRONT, -90).shootUp(2)), Rule.SHOOT_TARGET,
                Severity.ERROR));
    }

    // ---- the planning margin: a tip with no POLLEN to spare ---------------------------------------------------------

    @Test
    public void aFirstTipWithExactlyThreePollenHasNoSpareAndIsAWarning() {
        // 3 NECTAR + 3 POLLEN tip the cell on a calibrated field, but a field that needs one more would not tip
        List<Issue> l = validate(AutoRoute.builder("exact").start(58, FRONT, -90).shootUp(3));
        assertEquals(l.toString(), 1, count(l, Rule.TIP_MARGIN, Severity.WARN));
        Issue issue = first(l, Rule.TIP_MARGIN, Severity.WARN);
        assertEquals(1, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("puts 3 POLLEN into the AUDIENCE cell, exactly the 3 that tip it"));
        assertTrue(issue.message, issue.message.contains("no spare"));
        assertTrue(issue.message, issue.message.contains("QA46"));
        assertTrue(issue.message, issue.message.contains("Plan 4 POLLEN"));
        assertEquals("only a warning: it tips on a calibrated field", 0, count(l, Rule.TIP_MARGIN, Severity.ERROR));
        assertTrue(RouteValidator.isRunnable(l));
    }

    @Test
    public void aFirstTipWithTheFourPreloadsHasOneSpareAndNoWarning() {
        // the route that fires the 4 pre-loads at the up cell: 3 tip it, 1 is spare
        List<Issue> l = validate(AutoRoute.builder("preloads").start(58, FRONT, -90).shootUp(4));
        assertEquals(l.toString(), 0, count(l, Rule.TIP_MARGIN, Severity.WARN));
        assertEquals(1, AutoRoute.builder("p").start(58, FRONT, -90).shootUp(4).build().analyze().minimumTipSpare());
    }

    @Test
    public void aSecondTipWithExactlyEightPollenHasNoSpareAndPlanningNineIsClean() {
        // after our own tip the rear cell is empty: 4 + 4 are exactly the 8 that tip it
        List<Issue> tight = validate(AutoRoute.builder("tight").start(58, FRONT, -90).shootUp(4)
                .shootSide(Side.REAR, 4).shootSide(Side.REAR, 4));
        List<Issue> margin = new ArrayList<Issue>();
        for (Issue i : tight) {
            if (i.rule == Rule.TIP_MARGIN) {
                margin.add(i);
            }
        }
        assertEquals(tight.toString(), 1, margin.size());
        assertEquals("on the volley that tips the second HIVE", 3, margin.get(0).stepIndex);
        assertTrue(margin.get(0).message, margin.get(0).message.contains("puts 8 POLLEN into the REAR cell, exactly the 8 that tip it"));
        assertTrue(margin.get(0).message, margin.get(0).message.contains("Plan 9 POLLEN"));
        // 4 + 3 + 2 puts 9 into it: the 8th tips it and the 9th is the spare
        List<Issue> nine = validate(AutoRoute.builder("nine").start(58, FRONT, -90).shootUp(4)
                .shootSide(Side.REAR, 4).shootSide(Side.REAR, 3).shootSide(Side.REAR, 2));
        assertEquals(nine.toString(), 0, count(nine, Rule.TIP_MARGIN, Severity.WARN));
    }

    @Test
    public void thePartnersOwnTipIsNotOurSpareToWorryAbout() {
        // a route that only waits for the partner's tip tips nothing itself: the margin rule has nothing to judge
        List<Issue> l = validate(partnerRear().waitForTip(20));
        assertEquals(l.toString(), 0, count(l, Rule.TIP_MARGIN, Severity.WARN));
    }

    // ---- G417: the partner's tip and the cell that comes up ---------------------------------------------

    @Test
    public void waitingForTheTipAndThenShootingTheRearCellIsClean() {
        // give up at 20 s: the tip is assumed at 12 s (8 s of slack) and the volley after it needs about 6 s, so the
        // route is over by about 18 s, well inside the 28 s budget
        List<Issue> l = validate(thenLeave(partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4)));
        assertTrue("no findings expected, got " + l, l.isEmpty());
        // the same with the cell named instead of "the UP cell"
        assertTrue(validate(thenLeave(partnerRear().waitForTip(20).shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4))).isEmpty());
    }

    @Test
    public void shootingTheCellThatComesUpBeforeThePartnerTipsWithoutWaitingIsAnError() {
        List<Issue> l = validate(partnerRear().shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.ERROR);
        assertEquals(1, issue.stepIndex);
        assertEquals("an ERROR, not the WARN that a cell that is DOWN gets in a route that shoots the first tip itself", 0,
                count(l, Rule.SHOOT_TARGET, Severity.WARN));
        assertTrue(issue.message, issue.message.contains("REAR cell"));
        assertTrue(issue.message, issue.message.contains("partner's tip"));
        assertTrue(issue.message, issue.message.contains("G417"));
        assertTrue(issue.message, issue.message.contains("WAIT_FOR_TIP"));
        assertTrue(issue.message, issue.message.contains(String.format(java.util.Locale.ROOT, "%.1f s", TIP_SEC)));
        assertFalse("an ERROR means the route cannot be run as written", RouteValidator.isRunnable(l));
    }

    @Test
    public void aWaitThatEndsBeforeTheTipDoesNotMakeTheShootLegalAndTheMessageSaysSo() {
        List<Issue> l = validate(partnerRear().waitForTip(3).shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.ERROR);
        assertTrue(issue.message, issue.message.contains("WAIT_FOR_TIP at step 2"));
        assertTrue(issue.message, issue.message.contains("gives up at 3.0 s"));
        assertTrue(issue.message, issue.message.contains("raise the give-up time to at least " + CONFIRMED));
        assertTrue("the robot would skip the volley", issue.message.contains("skip this volley"));
        // and the wait itself is reported: it gives up before the tip is confirmed
        Issue wait = first(l, Rule.TIP_ASSUMPTION, Severity.WARN);
        assertEquals(1, wait.stepIndex);
        assertTrue(wait.message, wait.message.contains("gives up"));
        assertTrue(wait.message, wait.message.contains("at least " + CONFIRMED));
    }

    @Test
    public void aWaitThatEndsAtTheTipButBeforeTheCameraHasSeenItSteadyIsStillAnError() {
        // The settle time is required: a wait that gives up exactly at the assumed tip (12 s) ends 0.5 s before the camera can have
        // seen the cell steady, so the robot would go on to a cell it has not confirmed. Without the settle time in the model this
        // route would be clean.
        List<Issue> l = validate(partnerRear().waitForTip(TIP_SEC).shootUp(REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.ERROR);
        assertTrue(issue.message, issue.message.contains("confirmed"));
        assertTrue(issue.message, issue.message.contains("the camera must see the cell steady first"));
        assertTrue(issue.message, issue.message.contains(CONFIRMED));
        // one settle time later it is fine
        assertEquals(0, count(validate(partnerRear().waitForTip(TIP_SEC + SETTLE).shootUp(REAR_X, REAR_Y, 90, 4)), Rule.SHOOT_TARGET,
                Severity.ERROR));
    }

    @Test
    public void aShootAtTheCellThatComesUpNeedsAWaitForTipEvenAfterTheAssumedTipTime() {
        // A plain 13 s WAIT is no WAIT_FOR_TIP. The shoot starts after the assumed tip (12 s), so by the CLOCK the cell is
        // up, but partner teams tip at different times and the robot must not guess: only its camera may let it shoot.
        List<Issue> l = validate(partnerRear().waitSeconds(TIP_SEC + 1.0).shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.ERROR);
        assertEquals("the shoot is step 3", 2, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("no WAIT_FOR_TIP before it"));
        assertTrue(issue.message, issue.message.contains("camera"));
        assertTrue(issue.message, issue.message.contains("never because a time has passed"));
        assertFalse(RouteValidator.isRunnable(l));
        // "the UP cell" is the same cell by then, so shootUp is just as wrong
        List<Issue> up = validate(partnerRear().waitSeconds(TIP_SEC + 1.0).shootUp(REAR_X, REAR_Y, 90, 4));
        assertEquals(up.toString(), 1, count(up, Rule.SHOOT_TARGET, Severity.ERROR));
        // the same route with a WAIT_FOR_TIP instead of the plain wait is fine
        assertEquals(0, count(validate(partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4)), Rule.SHOOT_TARGET, Severity.ERROR));
    }

    @Test
    public void aWaitForTipAfterTheShootDoesNotCountBecauseItComesTooLate() {
        List<Issue> l = validate(partnerRear().shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4).waitForTip(20));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        assertEquals(1, first(l, Rule.SHOOT_TARGET, Severity.ERROR).stepIndex);
    }

    @Test
    public void everyShootAtTheComingUpCellNeedsAWaitNotOnlyTheFirst() {
        // one WAIT_FOR_TIP before the first volley covers the later volleys too: the cell stays up after it
        List<Issue> l = validate(partnerRear().waitForTip(18).shootUp(REAR_X, REAR_Y, 90, 4)
                .intakeFlower(BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][1], IntakeWheel.LEFT, 4).shootUp(REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
    }

    @Test
    public void aRouteThatShootsTheFirstTipItselfNeedsNoWait() {
        // SELF: no partner tip, so no wait: the second volley goes into the cell that came up with OUR tip (the robot
        // is camera-gated on the real field anyway, but the plan has no outside event to wait for)
        List<Issue> l = validate(AutoRoute.builder("self").start(58, FRONT, -90).shootUp(4).shootUp(4));
        assertEquals(l.toString(), 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        assertEquals(0, count(l, Rule.TIP_ASSUMPTION, Severity.WARN));
    }

    @Test
    public void aRouteThatShootsTheFirstTipItselfKeepsTheWarnForACellThatIsDown() {
        // SELF: the REAR cell is DOWN at the start and nobody tips it; the old WARN stays a WARN
        List<Issue> l = validate(AutoRoute.builder("self").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.WARN));
        assertEquals(0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        assertTrue(first(l, Rule.SHOOT_TARGET, Severity.WARN).message.contains("AUDIENCE cell is the UP one"));
    }

    @Test
    public void aVolleyAtTheOldCellThatIsStillFiringWhenThePartnerTipsItIsAWarning() {
        // 10.5 s of waiting, then 4 POLLEN at the UP (audience) cell: the volley runs from about 11.3 s to 16.8 s and
        // the partner is assumed to tip that very cell at 12 s
        List<Issue> l = validate(AutoRoute.builder("race").start(58, FRONT, -90).partnerTipsFirst().waitSeconds(10.5).shootUp(4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.WARN));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.WARN);
        assertEquals(2, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("tipping down"));
        // a volley that is over before the tip is fine: 2 POLLEN at the audience cell starting at 0 s end at about 4.5 s
        List<Issue> early = validate(AutoRoute.builder("early").start(58, FRONT, -90).partnerTipsFirst().shootSide(Side.AUDIENCE, 2));
        assertEquals(early.toString(), 0, count(early, Rule.SHOOT_TARGET, Severity.WARN));
    }

    // ---- the assumption must hold ------------------------------------------------------------------------

    @Test
    public void aWaitForTipInARouteThatShootsTheFirstTipItselfIsAWarning() {
        List<Issue> self = validate(AutoRoute.builder("self").start(58, FRONT, -90).waitForTip(5));
        assertEquals(self.toString(), 1, count(self, Rule.TIP_ASSUMPTION, Severity.WARN));
        Issue issue = first(self, Rule.TIP_ASSUMPTION, Severity.WARN);
        assertEquals(1, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("WE shoot the first tip"));
        assertEquals("only a warning: the step does nothing", 0, count(self, Rule.TIP_ASSUMPTION, Severity.ERROR));
        // the same wait in a partner route is what it is for
        List<Issue> partner = validate(AutoRoute.builder("partner").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitForTip(20));
        assertEquals(partner.toString(), 0, count(partner, Rule.TIP_ASSUMPTION, Severity.WARN));
    }

    // ---- the give-up time and the slack of a partner route ------------------------------------------------

    @Test
    public void aGiveUpTimeLaterThanTheRestOfTheRouteCanAffordIsAWarningThatNamesTheLatestOne() {
        // the volley after the wait takes about 6 s, the route must be over by 28 s: waiting until 25 s could overrun
        List<Issue> l = validate(thenLeave(partnerRear().waitForTip(25).shootUp(REAR_X, REAR_Y, 90, 4)));
        List<Issue> late = new ArrayList<Issue>();
        for (Issue i : l) {
            if (i.rule == Rule.TIP_ASSUMPTION && i.message.contains("run past the AUTO limit")) {
                late.add(i);
            }
        }
        assertEquals(l.toString(), 1, late.size());
        assertEquals(Severity.WARN, late.get(0).severity);
        assertEquals(1, late.get(0).stepIndex);
        assertTrue(late.get(0).message, late.get(0).message.contains("gives up at 25.0 s"));
        // and the message says which time would be fine: the same number the analysis reports
        RouteAnalysis a = thenLeave(partnerRear().waitForTip(25).shootUp(REAR_X, REAR_Y, 90, 4)).build().analyze();
        double latest = a.latestGiveUpSec(a.step(1));
        assertTrue(late.get(0).message, late.get(0).message.contains(String.format(java.util.Locale.ROOT, "%.1f s", latest)));
        assertEquals("only a warning: the robot cuts the wait there by itself", 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        // giving up at exactly the latest time is fine
        List<Issue> ok = validate(thenLeave(partnerRear().waitForTip(Math.floor(latest * 10) / 10).shootUp(REAR_X, REAR_Y, 90, 4)));
        assertEquals(ok.toString(), 0, count(ok, Rule.TIP_ASSUMPTION, Severity.WARN));
    }

    @Test
    public void aPartnerRouteWithLittleSlackIsAWarning() {
        // gives up at 13 s: the tip is assumed at 12 s, so a partner that is 1 s slower already breaks the route
        List<Issue> l = validate(thenLeave(partnerRear().waitForTip(13).shootUp(REAR_X, REAR_Y, 90, 4)));
        assertEquals(l.toString(), 1, count(l, Rule.TIP_ASSUMPTION, Severity.WARN));
        Issue issue = first(l, Rule.TIP_ASSUMPTION, Severity.WARN);
        assertEquals("on the WAIT_FOR_TIP", 1, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("latest tip this route can still use is 12.5 s"));
        assertTrue(issue.message, issue.message.contains("0.5 s of slack"));
        assertTrue(issue.message, issue.message.contains("the WAIT_FOR_TIP gives up at 13.0 s"));
        assertEquals("a warning, not an error: it works with the assumed tip", 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        // zero slack: the wait gives up at the very moment the assumed tip is confirmed
        List<Issue> zero = validate(thenLeave(partnerRear().waitForTip(TIP_SEC + SETTLE).shootUp(REAR_X, REAR_Y, 90, 4)));
        assertEquals(zero.toString(), 1, count(zero, Rule.TIP_ASSUMPTION, Severity.WARN));
        assertTrue(first(zero, Rule.TIP_ASSUMPTION, Severity.WARN).message.contains("0.0 s of slack"));
    }

    @Test
    public void theSlackWarningStartsExactlyBelowTheMinimum() {
        double min = BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC;
        // enough slack: the latest tip is exactly `min` seconds after the assumed one, so the wait may give up the settle time later
        assertEquals(0, count(validate(thenLeave(partnerRear().waitForTip(TIP_SEC + SETTLE + min).shootUp(REAR_X, REAR_Y, 90, 4))), Rule.TIP_ASSUMPTION,
                Severity.WARN));
        // a quarter of a second less is too little
        assertEquals(1, count(validate(thenLeave(partnerRear().waitForTip(TIP_SEC + SETTLE + min - 0.25).shootUp(REAR_X, REAR_Y, 90, 4))), Rule.TIP_ASSUMPTION,
                Severity.WARN));
    }

    @Test
    public void aSlowRouteAfterTheWaitUsesUpTheSlackBecauseTheAutoLimitComesFirst() {
        // After the tip the route fires 4, fetches 4 from the rear FLOWER and fires again: about 20 s of work. The wait may
        // give up as late as 20 s, but the work after it only fits if the tip comes by about 8 s, which the assumed 12 s
        // misses: no slack at all, whatever the give-up time says.
        AutoRoute.Builder slow = partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4)
                .intakeFlower(BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][1], IntakeWheel.LEFT, 4).shootUp(REAR_X, REAR_Y, 90, 4);
        List<Issue> l = validate(slow);
        RouteAnalysis a = slow.build().analyze();
        assertTrue("the budget, not the give-up time, is the limit: " + a.latestPartnerTipSec(), a.latestPartnerTipSec() < 20.0);
        boolean sawSlack = false;
        for (Issue i : l) {
            if (i.rule == Rule.TIP_ASSUMPTION && i.message.contains("slack")) {
                sawSlack = true;
                assertTrue(i.message, i.message.contains("the steps after the wait take"));
            }
        }
        assertTrue(l.toString(), sawSlack);
    }

    @Test
    public void aPartnerRouteThatDoesNotDependOnTheTipHasNothingToWarnAbout() {
        // it just walks off the rear wall and never shoots at the cell that comes up: any partner time is fine
        List<Issue> l = validate(partnerRear().driveConstant(61.5, 120, 90));
        assertEquals(l.toString(), 0, count(l, Rule.TIP_ASSUMPTION, Severity.WARN));
        assertEquals(0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
    }

    @Test
    public void tippingTheHiveOurselvesBeforeThePartnersAssumedTimeIsAWarning() {
        // 4 pre-loads at the start: our tip is at about 6 s, before the assumed 12 s
        List<Issue> l = validate(AutoRoute.builder("early").start(58, FRONT, -90).partnerTipsFirst().shootUp(4));
        assertEquals(l.toString(), 1, count(l, Rule.TIP_ASSUMPTION, Severity.WARN));
        Issue issue = first(l, Rule.TIP_ASSUMPTION, Severity.WARN);
        assertEquals("on the volley that tips", 1, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("before the partner's tip"));
        // the same route written for a robot that shoots the first tip itself has no such finding
        assertEquals(0, count(validate(AutoRoute.builder("self").start(58, FRONT, -90).shootUp(4)), Rule.TIP_ASSUMPTION,
                Severity.WARN));
    }

    // ---- G409: stay out of the spill zone -----------------------------------------------------------------

    @Test
    public void standingUnderTheAudienceCellWhenThePartnerTipsItIsAnError() {
        // C1 start, drive to y = 39 (the footprint reaches y 47.0: the BACK edge is 8.03 in behind the center) and wait there, in the middle of the spill zone
        // (y 30.67 to 75.05: from where the tipped cell's base hangs out to where its balls land, 40 in out from the pivot line)
        List<Issue> l = validate(AutoRoute.builder("zone").start(58, FRONT, -90).partnerTipsFirst()
                .driveConstant(58, 39, -90).waitSeconds(15));
        assertEquals(l.toString(), 1, count(l, Rule.SPILL_G409, Severity.ERROR));
        Issue issue = first(l, Rule.SPILL_G409, Severity.ERROR);
        assertEquals("the wait is step 3", 2, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("AUDIENCE cell"));
        assertTrue(issue.message, issue.message.contains("partner's assumed tip"));
        assertTrue(issue.message, issue.message.contains("G409"));
        assertEquals("nothing else is wrong with that route", 1, l.size());
    }

    @Test
    public void standingOutsideTheSpillZoneWhenThePartnerTipsIsFine() {
        // y = 14.8, the height of the standard shooting pose: the rear edge is at y 22.8, 47.8 in out from the pivot line, past the 40 in the balls reach
        // (the zone starts at y 30.67); and the rear robot is on the other side
        assertEquals(0, count(validate(AutoRoute.builder("clear").start(58, FRONT, -90).partnerTipsFirst()
                .driveConstant(58, 14.8, -90).waitSeconds(15)), Rule.SPILL_G409, Severity.ERROR));
        assertEquals(0, count(validate(partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4)), Rule.SPILL_G409,
                Severity.ERROR));
    }

    /**
     * Situation: a robot waits for the partner's tip (assumed at 12 s) facing the audience wall at different heights, its rear edge (8.03 in behind
     *     its center) toward the HIVE.
     * Expected: at y = 14.8 and 20 (rear edge 47.8 and 42.6 in out from the pivot line, which is 70.67 - y - 8.03) it is outside the zone; at y = 26, 30, 32
     *     and 36 (rear edge 36.6 to 26.6 in out) it is INSIDE it: an ERROR of rule G409 and nothing else, in particular NOT the HIVE keep-out (which
     *     starts 22.5 in out).
     * Why: the first version of the zone was the cell's footprint plus 6 in, which reaches only 26 in out. The balls of a tipped cell first touch the
     *     floor 31 to 35 in out (field report hive_tags, section e), so a robot standing at y = 26 to 32, which the old zone called fine, has the balls
     *     land in front of its rear face, or on it: exactly the catch or deflection that G409 forbids. Here every one of those heights is an error.
     * Negative twin: the old zone (the cell's footprint grown by TIP_SPILL_MARGIN_IN) does not overlap the footprint at y = 26 to 32, so a check built
     *     on it would have reported these routes clean.
     */
    @Test
    public void aRobotStandingWhereTheBallsOfTheTippedCellLandIsAnErrorNotJustOneUnderTheCell() {
        double[] clear = {14.8, 20.0};
        double[] inLandingBand = {26.0, 30.0, 32.0, 36.0};
        for (double y : clear) {
            List<Issue> l = validate(AutoRoute.builder("band").start(58, FRONT, -90).partnerTipsFirst().driveConstant(58, y, -90).waitSeconds(15));
            assertEquals("y = " + y + ": " + l, 0, count(l, Rule.SPILL_G409, Severity.ERROR));
        }
        AllianceField f = new AllianceField(new FieldModel(), Alliance.RED);
        Rect oldZone = f.cellPlan(Alliance.RED, Side.AUDIENCE, true).inflate(BioBuzzConstants.Auto.TIP_SPILL_MARGIN_IN);
        for (double y : inLandingBand) {
            List<Issue> l = validate(AutoRoute.builder("band").start(58, FRONT, -90).partnerTipsFirst().driveConstant(58, y, -90).waitSeconds(15));
            assertEquals("y = " + y + ": " + l, 1, count(l, Rule.SPILL_G409, Severity.ERROR));
            assertEquals("only the spill rule reports it (the HIVE keep-out starts farther in): " + l, 1, l.size());
            // the old zone is 26 in out from the pivot line; the footprint's rear edge (y + 8.03) does not reach it at y <= 32 (rear edge y 40.03 < 44.61)
            if (y <= 32.0) {
                assertFalse("y = " + y, Footprint.of(new Pose2(58, y, -Math.PI / 2.0)).overlaps(oldZone));
            }
        }
    }

    /**
     * Situation: the spill zone of the tipped cell, asked for both alliances and both cells.
     * Expected: for the red AUDIENCE cell x [41.39, 74.45], y [30.67, 75.05] (the cell's width plus the 6 in margin; from 4.4 in past the pivot line to
     *     TIP_SPILL_REACH_IN, 40 in, out from it); for the red REAR cell the mirror image about the pivot line; blue is the half turn of red; and the
     *     zone covers the tipped cell's own footprint.
     */
    @Test
    public void theSpillZoneIsTheCellsWidthFromJustPastThePivotLineOutToWhereTheBallsLand() {
        AllianceField red = new AllianceField(new FieldModel(), Alliance.RED);
        double center = BioBuzzConstants.Field.CENTER_IN;
        Rect audience = red.spillZone(Alliance.RED, Side.AUDIENCE);
        assertEquals(41.39, audience.minX, 0.01);
        assertEquals(74.45, audience.maxX, 0.01);
        assertEquals(center - BioBuzzConstants.Auto.TIP_SPILL_REACH_IN, audience.minY, 1e-9);
        assertEquals(30.67, audience.minY, 0.01);
        assertEquals(75.05, audience.maxY, 0.01);
        Rect rear = red.spillZone(Alliance.RED, Side.REAR);
        assertEquals(audience.minX, rear.minX, 1e-9);
        assertEquals(2.0 * center - audience.maxY, rear.minY, 1e-9);
        assertEquals(2.0 * center - audience.minY, rear.maxY, 1e-9);
        Rect blueRear = red.spillZone(Alliance.BLUE, Side.REAR);
        Rect expectedBlue = audience.rotated180(center, center);
        assertEquals(expectedBlue.minX, blueRear.minX, 1e-9);
        assertEquals(expectedBlue.maxX, blueRear.maxX, 1e-9);
        assertEquals(expectedBlue.minY, blueRear.minY, 1e-9);
        assertEquals(expectedBlue.maxY, blueRear.maxY, 1e-9);
        // it covers the cell's own footprint, before and after the tip
        for (boolean up : new boolean[] {true, false}) {
            Rect cell = red.cellPlan(Alliance.RED, Side.AUDIENCE, up);
            assertTrue(up + ": " + cell, audience.minX <= cell.minX && audience.maxX >= cell.maxX && audience.minY <= cell.minY && audience.maxY >= cell.maxY);
        }
    }

    /**
     * Situation: the standard shooting poses a route uses, each held while the cell they just tipped spills: the nominal pose the solver picks (34 in
     *     from the aim point), the measured anchor shot (front on the audience wall, 37.7 in) and the closest standoff the validator accepts (30 in).
     * Expected: all of them are OUTSIDE the zone (their rear edge is about 48, 53 and 45 in out); a pose 20 in from the aim point, which the validator
     *     already calls too close, is inside.
     * Why: this is the decision the zone had to make, written as a test: the robot's shooter faces the HIVE with its BACK, so the rear edge is what the
     *     balls would reach. A zone that flagged the standard shot would flag every route; the day-one checklist asks the team to look for the balls
     *     there, and TIP_SPILL_REACH_IN is the knob if they do reach it. With 40 in no pose of the legal shooting band is in the zone.
     */
    @Test
    public void everyLegalShootingPoseIsOutsideTheSpillZoneAndOneMuchCloserIsInside() {
        AllianceField f = new AllianceField(new FieldModel(), Alliance.RED);
        Rect zone = f.spillZone(Alliance.RED, Side.AUDIENCE);
        CellSpec cell = f.cell(Side.AUDIENCE);
        Pose2 nominal = f.nominalShootPose(Side.AUDIENCE);
        assertFalse("nominal pose " + nominal, Footprint.of(nominal).overlaps(zone));
        double nominalRearOut = BioBuzzConstants.Field.CENTER_IN - Footprint.of(nominal).bounds().maxY;
        assertTrue("rear edge " + nominalRearOut + " in out", nominalRearOut > BioBuzzConstants.Auto.TIP_SPILL_REACH_IN);
        // a pose on the HIVE axis facing the audience wall whose ball exit is {standoff} from the aim point: the exit is 6.58 in BEHIND the center of rotation
        for (double standoff : new double[] {BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN, BioBuzzConstants.Auto.SHOOT_STANDOFF_NOMINAL_IN,
                BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN, 37.7}) {
            Pose2 p = new Pose2(cell.originX, cell.originY - standoff + BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, -Math.PI / 2.0);
            assertFalse("standoff " + standoff + " at " + p, Footprint.of(p).overlaps(zone));
        }
        Pose2 tooClose = new Pose2(cell.originX, cell.originY - 20.0 + BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, -Math.PI / 2.0);
        assertTrue("20 in from the aim point " + tooClose, Footprint.of(tooClose).overlaps(zone));
    }

    @Test
    public void theSpillRuleAlsoCoversTheWholeTimeAfterTheRouteHasEnded() {
        // the route is over at about 2 s, the assumed tip is at 12 s: the robot still stands in the zone then
        List<Issue> l = validate(AutoRoute.builder("end").start(58, FRONT, -90).partnerTipsFirst().driveConstant(58, 39, -90));
        assertEquals(l.toString(), 1, count(l, Rule.SPILL_G409, Severity.ERROR));
        assertEquals("on the last step", 1, first(l, Rule.SPILL_G409, Severity.ERROR).stepIndex);
        assertEquals(0, count(validate(AutoRoute.builder("end2").start(58, FRONT, -90).partnerTipsFirst()
                .driveConstant(58, 14.8, -90)), Rule.SPILL_G409, Severity.ERROR));
    }

    @Test
    public void drivingIntoTheZoneRightAfterOurOwnTipIsAnErrorButWaitingForTheSpillToEndIsFine() {
        // our volley tips the HIVE; the next leg drives toward the cell and reaches the zone within the 3 s
        List<Issue> l = validate(AutoRoute.builder("own").start(58, FRONT, -90).shootUp(4).driveConstant(58, 39, -90));
        assertEquals(l.toString(), 1, count(l, Rule.SPILL_G409, Severity.ERROR));
        Issue issue = first(l, Rule.SPILL_G409, Severity.ERROR);
        assertEquals("the drive is step 3", 2, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("the HIVE tip of step 2"));
        // the same drive after waiting out the spill time: the robot only gets there later
        List<Issue> late = validate(AutoRoute.builder("late").start(58, FRONT, -90).shootUp(4)
                .waitSeconds(BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC + 0.5).driveConstant(58, 39, -90));
        assertEquals(late.toString(), 0, count(late, Rule.SPILL_G409, Severity.ERROR));
    }

    @Test
    public void theShootingPoseItselfIsOutsideTheSpillZoneOfTheCellItTips() {
        // standing at the nominal standoff pose (34 in from the aim point) while the cell spills is the normal case
        assertEquals(0, count(validate(AutoRoute.builder("shoot").start(58, FRONT, -90).shootUp(4).waitSeconds(5)),
                Rule.SPILL_G409, Severity.ERROR));
    }
}
