package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The validator rules about HIVE tips (G417 and G409 and the first-tip assumption), each with a route that breaks it
 * and a legal one next to it. Every route is validated for RED and for its BLUE half turn and both must agree.
 */
public class TipRulesTest {
    private static final double TIP_SEC = BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC;

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
        return AutoRoute.builder("partner_rear").start(61.5, 135.1, 90).partnerTipsFirst();
    }

    private static final double REAR_X = 62.0;
    private static final double REAR_Y = 132.5;

    // ---- G417: POLLEN fired after the tip ---------------------------------------------------------------

    @Test
    public void aVolleyThatKeepsFiringAfterTheTipIsAnError() {
        // 2 POLLEN, take 2 back from the floor, then fire 4: the 2nd of those 4 is the tip, the other 2 would go into
        // a cell that is already tipping down
        List<Issue> l = validate(AutoRoute.builder("late").start(58, 9.2, -90)
                .shootUp(2).intakeFloor(60, 30, 59.25, 30, -90, 2).shootUp(4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.ERROR);
        assertEquals("the second volley is step 4", 3, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("tips with POLLEN 2 of the 4"));
        assertTrue(issue.message, issue.message.contains("the last 2"));
        assertTrue(issue.message, issue.message.contains("G417"));
    }

    @Test
    public void aVolleyWhoseLastPollenIsTheTipIsFine() {
        // 4 at once: the 4th POLLEN tips, nothing is fired after it
        assertEquals(0, count(validate(AutoRoute.builder("one").start(58, 9.2, -90).shootUp(4)), Rule.SHOOT_TARGET, Severity.ERROR));
        // 2 + 2 into the same cell: the last POLLEN of the second volley tips
        assertEquals(0, count(validate(AutoRoute.builder("two").start(58, 9.2, -90).shootUp(2).shootUp(2)), Rule.SHOOT_TARGET,
                Severity.ERROR));
        // 3 POLLEN never reach the tip: nothing to complain about either
        assertEquals(0, count(validate(AutoRoute.builder("three").start(58, 9.2, -90).shootUp(3)), Rule.SHOOT_TARGET,
                Severity.ERROR));
    }

    // ---- G417: the partner's tip and the cell that comes up ---------------------------------------------

    @Test
    public void waitingForTheTipAndThenShootingTheRearCellIsClean() {
        // give up at 20 s: the tip is assumed at 12 s (8 s of slack) and the volley after it needs about 6 s, so the
        // route is over by about 18 s, well inside the 28 s budget
        List<Issue> l = validate(partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4));
        assertTrue("no findings expected, got " + l, l.isEmpty());
        // the same with the cell named instead of "the UP cell"
        assertTrue(validate(partnerRear().waitForTip(20).shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4)).isEmpty());
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
        assertTrue(issue.message, issue.message.contains("raise the give-up time"));
        assertTrue("the robot would skip the volley", issue.message.contains("skip this volley"));
        // and the wait itself is reported: it gives up before the tip
        Issue wait = first(l, Rule.TIP_ASSUMPTION, Severity.WARN);
        assertEquals(1, wait.stepIndex);
        assertTrue(wait.message, wait.message.contains("gives up"));
        assertTrue(wait.message, wait.message.contains("at least " + String.format(java.util.Locale.ROOT, "%.1f s", TIP_SEC)));
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
                .intakeFlower(48, 141.6, 48, 133, 90, 4).shootUp(REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
    }

    @Test
    public void aRouteThatShootsTheFirstTipItselfNeedsNoWait() {
        // SELF: no partner tip, so no wait: the second volley goes into the cell that came up with OUR tip (the robot
        // is camera-gated on the real field anyway, but the plan has no outside event to wait for)
        List<Issue> l = validate(AutoRoute.builder("self").start(58, 9.2, -90).shootUp(4).shootUp(4));
        assertEquals(l.toString(), 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        assertEquals(0, count(l, Rule.TIP_ASSUMPTION, Severity.WARN));
    }

    @Test
    public void aRouteThatShootsTheFirstTipItselfKeepsTheWarnForACellThatIsDown() {
        // SELF: the REAR cell is DOWN at the start and nobody tips it; the old WARN stays a WARN
        List<Issue> l = validate(AutoRoute.builder("self").start(61.5, 135.1, 90).shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.WARN));
        assertEquals(0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        assertTrue(first(l, Rule.SHOOT_TARGET, Severity.WARN).message.contains("AUDIENCE cell is the UP one"));
    }

    @Test
    public void aVolleyAtTheOldCellThatIsStillFiringWhenThePartnerTipsItIsAWarning() {
        // 10.5 s of waiting, then 4 POLLEN at the UP (audience) cell: the volley runs from about 11.3 s to 16.8 s and
        // the partner is assumed to tip that very cell at 12 s
        List<Issue> l = validate(AutoRoute.builder("race").start(58, 9.2, -90).partnerTipsFirst().waitSeconds(10.5).shootUp(4));
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.WARN));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.WARN);
        assertEquals(2, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("tipping down"));
        // a volley that is over before the tip is fine: 2 POLLEN at the audience cell starting at 0 s end at about 4.5 s
        List<Issue> early = validate(AutoRoute.builder("early").start(58, 9.2, -90).partnerTipsFirst().shootSide(Side.AUDIENCE, 2));
        assertEquals(early.toString(), 0, count(early, Rule.SHOOT_TARGET, Severity.WARN));
    }

    // ---- the assumption must hold ------------------------------------------------------------------------

    @Test
    public void aWaitForTipInARouteThatShootsTheFirstTipItselfIsAWarning() {
        List<Issue> self = validate(AutoRoute.builder("self").start(58, 9.2, -90).waitForTip(5));
        assertEquals(self.toString(), 1, count(self, Rule.TIP_ASSUMPTION, Severity.WARN));
        Issue issue = first(self, Rule.TIP_ASSUMPTION, Severity.WARN);
        assertEquals(1, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("WE shoot the first tip"));
        assertEquals("only a warning: the step does nothing", 0, count(self, Rule.TIP_ASSUMPTION, Severity.ERROR));
        // the same wait in a partner route is what it is for
        List<Issue> partner = validate(AutoRoute.builder("partner").start(61.5, 135.1, 90).partnerTipsFirst().waitForTip(20));
        assertEquals(partner.toString(), 0, count(partner, Rule.TIP_ASSUMPTION, Severity.WARN));
    }

    // ---- the give-up time and the slack of a partner route ------------------------------------------------

    @Test
    public void aGiveUpTimeLaterThanTheRestOfTheRouteCanAffordIsAWarningThatNamesTheLatestOne() {
        // the volley after the wait takes about 6 s, the route must be over by 28 s: waiting until 25 s could overrun
        List<Issue> l = validate(partnerRear().waitForTip(25).shootUp(REAR_X, REAR_Y, 90, 4));
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
        RouteAnalysis a = partnerRear().waitForTip(25).shootUp(REAR_X, REAR_Y, 90, 4).build().analyze();
        double latest = a.latestGiveUpSec(a.step(1));
        assertTrue(late.get(0).message, late.get(0).message.contains(String.format(java.util.Locale.ROOT, "%.1f s", latest)));
        assertEquals("only a warning: the robot cuts the wait there by itself", 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        // giving up at exactly the latest time is fine
        List<Issue> ok = validate(partnerRear().waitForTip(Math.floor(latest * 10) / 10).shootUp(REAR_X, REAR_Y, 90, 4));
        assertEquals(ok.toString(), 0, count(ok, Rule.TIP_ASSUMPTION, Severity.WARN));
    }

    @Test
    public void aPartnerRouteWithLittleSlackIsAWarning() {
        // gives up at 13 s: the tip is assumed at 12 s, so a partner that is 1 s slower already breaks the route
        List<Issue> l = validate(partnerRear().waitForTip(13).shootUp(REAR_X, REAR_Y, 90, 4));
        assertEquals(l.toString(), 1, count(l, Rule.TIP_ASSUMPTION, Severity.WARN));
        Issue issue = first(l, Rule.TIP_ASSUMPTION, Severity.WARN);
        assertEquals("on the WAIT_FOR_TIP", 1, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("latest tip this route can still use is 13.0 s"));
        assertTrue(issue.message, issue.message.contains("1.0 s of slack"));
        assertTrue(issue.message, issue.message.contains("the WAIT_FOR_TIP gives up at 13.0 s"));
        assertEquals("a warning, not an error: it works with the assumed tip", 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        // zero slack: the wait gives up at the very moment of the assumed tip
        List<Issue> zero = validate(partnerRear().waitForTip(TIP_SEC).shootUp(REAR_X, REAR_Y, 90, 4));
        assertEquals(zero.toString(), 1, count(zero, Rule.TIP_ASSUMPTION, Severity.WARN));
        assertTrue(first(zero, Rule.TIP_ASSUMPTION, Severity.WARN).message.contains("0.0 s of slack"));
    }

    @Test
    public void theSlackWarningStartsExactlyBelowTheMinimum() {
        double min = BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC;
        // enough slack: give up exactly `min` seconds after the assumed tip
        assertEquals(0, count(validate(partnerRear().waitForTip(TIP_SEC + min).shootUp(REAR_X, REAR_Y, 90, 4)), Rule.TIP_ASSUMPTION,
                Severity.WARN));
        // a quarter of a second less is too little
        assertEquals(1, count(validate(partnerRear().waitForTip(TIP_SEC + min - 0.25).shootUp(REAR_X, REAR_Y, 90, 4)), Rule.TIP_ASSUMPTION,
                Severity.WARN));
    }

    @Test
    public void aSlowRouteAfterTheWaitUsesUpTheSlackBecauseTheAutoLimitComesFirst() {
        // After the tip the route fires 4, fetches 4 from the rear FLOWER and fires again: about 20 s of work. The wait may
        // give up as late as 20 s, but the work after it only fits if the tip comes by about 8 s, which the assumed 12 s
        // misses: no slack at all, whatever the give-up time says.
        AutoRoute.Builder slow = partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4)
                .intakeFlower(48, 141.6, 48, 133, 90, 4).shootUp(REAR_X, REAR_Y, 90, 4);
        List<Issue> l = validate(slow);
        RouteAnalysis a = slow.build().analyze();
        assertTrue("the budget, not the give-up time, is the limit: " + a.latestPartnerTipSec(), a.latestPartnerTipSec() < 20.0);
        boolean sawSlack = false;
        for (Issue i : l) {
            if (i.rule == Rule.TIP_ASSUMPTION && i.message.contains("of slack")) {
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
        List<Issue> l = validate(AutoRoute.builder("early").start(58, 9.2, -90).partnerTipsFirst().shootUp(4));
        assertEquals(l.toString(), 1, count(l, Rule.TIP_ASSUMPTION, Severity.WARN));
        Issue issue = first(l, Rule.TIP_ASSUMPTION, Severity.WARN);
        assertEquals("on the volley that tips", 1, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("before the partner's tip"));
        // the same route written for a robot that shoots the first tip itself has no such finding
        assertEquals(0, count(validate(AutoRoute.builder("self").start(58, 9.2, -90).shootUp(4)), Rule.TIP_ASSUMPTION,
                Severity.WARN));
    }

    // ---- G409: stay out of the spill zone -----------------------------------------------------------------

    @Test
    public void standingUnderTheAudienceCellWhenThePartnerTipsItIsAnError() {
        // C1 start, drive to y = 38 (the footprint reaches y 46.9, the spill zone starts at 52.3 - 6 = 46.3) and wait there
        List<Issue> l = validate(AutoRoute.builder("zone").start(58, 9.2, -90).partnerTipsFirst()
                .driveConstant(58, 38, -90).waitSeconds(15));
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
        // y = 28: the footprint reaches y 36.9, nine inches short of the zone; and the rear robot is on the other side
        assertEquals(0, count(validate(AutoRoute.builder("clear").start(58, 9.2, -90).partnerTipsFirst()
                .driveConstant(58, 28, -90).waitSeconds(15)), Rule.SPILL_G409, Severity.ERROR));
        assertEquals(0, count(validate(partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4)), Rule.SPILL_G409,
                Severity.ERROR));
    }

    @Test
    public void theSpillRuleAlsoCoversTheWholeTimeAfterTheRouteHasEnded() {
        // the route is over at about 2 s, the assumed tip is at 12 s: the robot still stands in the zone then
        List<Issue> l = validate(AutoRoute.builder("end").start(58, 9.2, -90).partnerTipsFirst().driveConstant(58, 38, -90));
        assertEquals(l.toString(), 1, count(l, Rule.SPILL_G409, Severity.ERROR));
        assertEquals("on the last step", 1, first(l, Rule.SPILL_G409, Severity.ERROR).stepIndex);
        assertEquals(0, count(validate(AutoRoute.builder("end2").start(58, 9.2, -90).partnerTipsFirst()
                .driveConstant(58, 28, -90)), Rule.SPILL_G409, Severity.ERROR));
    }

    @Test
    public void drivingIntoTheZoneRightAfterOurOwnTipIsAnErrorButWaitingForTheSpillToEndIsFine() {
        // our volley tips the HIVE; the next leg drives toward the cell and reaches the zone within the 3 s
        List<Issue> l = validate(AutoRoute.builder("own").start(58, 9.2, -90).shootUp(4).driveConstant(58, 38, -90));
        assertEquals(l.toString(), 1, count(l, Rule.SPILL_G409, Severity.ERROR));
        Issue issue = first(l, Rule.SPILL_G409, Severity.ERROR);
        assertEquals("the drive is step 3", 2, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("the HIVE tip of step 2"));
        // the same drive after waiting out the spill time: the robot only gets there later
        List<Issue> late = validate(AutoRoute.builder("late").start(58, 9.2, -90).shootUp(4)
                .waitSeconds(BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC + 0.5).driveConstant(58, 38, -90));
        assertEquals(late.toString(), 0, count(late, Rule.SPILL_G409, Severity.ERROR));
    }

    @Test
    public void theShootingPoseItselfIsOutsideTheSpillZoneOfTheCellItTips() {
        // standing at the nominal standoff pose (34 in from the aim point) while the cell spills is the normal case
        assertEquals(0, count(validate(AutoRoute.builder("shoot").start(58, 9.2, -90).shootUp(4).waitSeconds(5)),
                Rule.SPILL_G409, Severity.ERROR));
    }
}
