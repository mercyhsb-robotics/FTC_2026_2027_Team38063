package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The plan side of "the camera decides, a time never does" (the user's fourth constraint): how a route is analysed when a volley
 * depends on a HIVE tip that only the camera can confirm.
 *
 * <ul>
 *   <li>the volley that follows OUR OWN tip waits for the camera to see the cell steady (the settle time), not for a clock;</li>
 *   <li>every volley that depends on a tip is known ({@code tipDependentSteps}), so the robot can wait for the camera before it and
 *       skip it when the camera cannot confirm the tip;</li>
 *   <li>the plan for "the tip is never confirmed" ({@code giveUpPlan}): the wait ends at its give-up time, the volleys are left out, the
 *       robot carries on, and the validator says when that loses the LEAVE or PARK the normal plan earns;</li>
 *   <li>the spill rule counts from the moment the camera has confirmed the tip.</li>
 * </ul>
 *
 * <p>How it works: no robot. Each test builds a small route, reads the analysis (the same simulation the validator and the gallery
 * use) and checks the numbers; the runtime side (the camera, the settle time, the deadline) is tested in {@code TipConfirmationTest}.
 */
public class TipConfirmationPlanTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double SETTLE = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
    private static final double TIP_SEC = BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC;
    private static final double EPS = 1e-9;
    private static final double REAR_X = 61.5;
    private static final double REAR_Y = 130.0;
    private static final double OFF_WALL_Y = 112.0;
    /**
     * The audience-side cell's spill zone starts at y 30.67 (it reaches 40 in out from the pivot line, y 70.67). A robot facing the audience wall has its
     * REAR edge 8.03 in behind its center, so at y = 22.5 the rear edge is at 30.53, 0.14 in short of the zone, and a step 1 in toward the cell is 0.86 in inside.
     */
    private static final double AT_THE_ZONE_Y = 22.5;

    private static AutoRoute.Builder partnerRear() {
        return AutoRoute.builder("partner_rear").start(REAR_X, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst();
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

    // ---- our own tip: the camera confirms it before the next volley ---------------------------------------------------

    @Test
    public void theVolleyAfterOurOwnTipWaitsForTheCameraToSeeTheNewCellSteady() {
        // Two volleys from the same pose, the second aimed at the cell that comes up with OUR tip: nothing separates them, so
        // without the settle time the second volley would start the moment the first tip lands.
        RouteAnalysis a = AutoRoute.builder("two").start(58, FRONT, -90)
                .shootUp(58, 30, -90, 4).shootSide(Side.REAR, 58, 30, -90, 2).build().analyze();
        RouteAnalysis.StepInfo first = a.step(1);
        RouteAnalysis.StepInfo second = a.step(2);
        assertTrue(first.tipped);
        assertEquals("the first volley is not waiting for any tip", 0.0, first.confirmWaitSec, EPS);
        assertFalse(first.tipDependent);
        assertEquals("the robot stands at its pose until the camera has seen the cell steady", SETTLE, second.confirmWaitSec, EPS);
        assertEquals("the first POLLEN of the second volley leaves no earlier than the confirmed tip", first.tipSec + SETTLE, second.launchSec, EPS);
        assertEquals("and the wait is part of the step", second.confirmWaitSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC
                + 2 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC, second.durationSec, EPS);
        assertTrue("it goes into the cell that is UP only because of our tip", second.tipDependent);
        assertTrue(second.targetWasUp);
    }

    @Test
    public void aRobotThatWasBusyLongEnoughHasNothingLeftToWaitFor() {
        // 3 s of other work between the tip and the next volley: the camera has long confirmed it, the volley starts at once
        RouteAnalysis a = AutoRoute.builder("two").start(58, FRONT, -90)
                .shootUp(58, 30, -90, 4).waitSeconds(3.0).shootSide(Side.REAR, 58, 30, -90, 2).build().analyze();
        assertEquals(0.0, a.step(3).confirmWaitSec, EPS);
        assertTrue("still tip-dependent: the camera confirmed it, but the volley relies on that", a.step(3).tipDependent);
        assertEquals(a.step(3).startSec, a.step(3).launchSec, EPS);
    }

    @Test
    public void aVolleyAtTheCellThatIsUpFromTheStartIsNotTipDependent() {
        RouteAnalysis a = AutoRoute.builder("first").start(58, FRONT, -90).shootUp(4).build().analyze();
        assertFalse(a.step(1).tipDependent);
        assertTrue(a.tipDependentSteps().isEmpty());
        assertFalse(a.dependsOnTipConfirmation());
        assertNull("nothing to give up on", a.giveUpPlan());
    }

    // ---- which volleys need the camera to confirm a tip -----------------------------------------------------------------

    @Test
    public void everyVolleyAtTheCellThatComesUpWithThePartnersTipIsTipDependent() {
        RouteAnalysis a = partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4).shootUp(REAR_X, REAR_Y, 90, 4).build().analyze();
        assertEquals(Arrays.asList(2, 3), a.tipDependentSteps());
        assertTrue(a.dependsOnTipConfirmation());
        // even a volley that starts before the assumed tip, aimed at the cell that comes up with it, depends on the tip (the validator ERRORs it)
        RouteAnalysis early = partnerRear().shootSide(Side.REAR, REAR_X, REAR_Y, 90, 4).build().analyze();
        assertEquals(Collections.singletonList(1), early.tipDependentSteps());
        // a volley at the cell that is UP when the route starts, in a partner route, is not: it does not need anybody's tip
        RouteAnalysis old = AutoRoute.builder("old").start(58, FRONT, -90).partnerTipsFirst().shootSide(Side.AUDIENCE, 2).build().analyze();
        assertTrue(old.tipDependentSteps().isEmpty());
    }

    // ---- the effective give-up time ------------------------------------------------------------------------------

    @Test
    public void theRobotStopsWaitingAtTheGiveUpTimeOrAtTheLatestTheRestAllowsWhicheverIsEarlier() {
        RouteAnalysis early = partnerRear().waitForTip(15).shootUp(REAR_X, REAR_Y, 90, 4).build().analyze();
        assertEquals("the deadline of the step comes first", 15.0, early.effectiveGiveUpSec(early.step(1)), EPS);
        RouteAnalysis late = partnerRear().waitForTip(28).shootUp(REAR_X, REAR_Y, 90, 4).build().analyze();
        assertTrue(late.latestGiveUpSec(late.step(1)) < 28.0);
        assertEquals("the AUTO limit comes first", late.latestGiveUpSec(late.step(1)), late.effectiveGiveUpSec(late.step(1)), EPS);
        assertTrue("only a WAIT_FOR_TIP has one", Double.isNaN(late.effectiveGiveUpSec(late.step(2))));
    }

    // ---- the plan for "the tip is never confirmed" ----------------------------------------------------------------------

    private static AutoRoute.Builder volleyThenLeave(double giveUp) {
        return partnerRear().waitForTip(giveUp).shootUp(REAR_X, REAR_Y, 90, 4).driveConstant(REAR_X, OFF_WALL_Y, 90);
    }

    @Test
    public void whenTheTipIsNeverConfirmedTheRobotWaitsLeavesTheVolleyOutAndStillLeaves() {
        RouteAnalysis plan = volleyThenLeave(20).build().analyze();
        RouteAnalysis g = plan.giveUpPlan();
        assertNotNull(g);
        assertSame("built once", g, plan.giveUpPlan());
        // the wait lasts until its deadline instead of until the tip
        assertEquals(plan.effectiveGiveUpSec(plan.step(1)), g.step(1).endSec(), EPS);
        assertTrue(g.step(1).endSec() > plan.step(1).endSec());
        // the volley is left out: no time, nothing fired, no tip, the robot has not moved
        RouteAnalysis.StepInfo volley = g.step(2);
        assertTrue(volley.skipped);
        assertEquals(0.0, volley.durationSec, EPS);
        assertEquals(volley.from.x, volley.to.x, EPS);
        assertEquals(volley.from.y, volley.to.y, EPS);
        assertEquals("the 4 pre-loads are still in the robot", BioBuzzConstants.Auto.PRELOAD_POLLEN, volley.pollenAfter);
        assertEquals(0, g.tips());
        assertEquals(0, g.pollenShot());
        assertTrue(g.tipEvents().isEmpty());
        // the drive off the wall starts from where the robot waited, not from the volley pose
        assertEquals(plan.step(0).to.y, g.step(3).from.y, EPS);
        assertTrue(g.leave());
        assertEquals("LEAVE is still earned, the safe points do not depend on the tip", BioBuzzConstants.Auto.LEAVE_POINTS, g.points());
        assertTrue(g.totalSeconds() <= RouteAnalysis.timeBudgetSec());
        assertFalse("none of the steps of the normal plan is marked skipped", plan.step(2).skipped);
    }

    @Test
    public void ourOwnSecondVolleyIsLeftOutTooWhenTheCameraCannotConfirmOurTip() {
        // our own second volley depends on our own tip: with a dead camera it is left out too (no camera, no shot)
        RouteAnalysis plan = AutoRoute.builder("two").start(58, FRONT, -90)
                .shootUp(58, 30, -90, 4).shootSide(Side.REAR, 58, 30, -90, 2).driveConstant(58, 20, -90).build().analyze();
        RouteAnalysis g = plan.giveUpPlan();
        assertNotNull(g);
        assertFalse("the first volley is at the cell that is up from the start, it goes ahead", g.step(1).skipped);
        assertTrue(g.step(1).tipped);
        assertTrue(g.step(2).skipped);
        assertEquals(plan.step(1).endSec(), g.step(2).startSec, EPS);
        assertEquals("only the first volley fires, and it stops at its tip", 3, g.pollenShot());
    }

    @Test
    public void anIntakeAfterALeftOutVolleyOnlyTakesWhatFitsInTheRobot() {
        // the robot still holds its 4 pre-loads because the volley was skipped: the rear FLOWER intake takes nothing (G407: never 5)
        double[] flower = BioBuzzConstants.Field.FLOWER_CENTERS_IN[1];
        RouteAnalysis plan = partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4)
                .intakeFlower(flower[0], flower[1], IntakeWheel.LEFT, 4).build().analyze();
        assertEquals("the normal plan takes 4 after firing 4", 4, plan.pollenGained());
        RouteAnalysis g = plan.giveUpPlan();
        assertEquals("the give-up plan holds 4 and takes none", 0, g.pollenGained());
        assertEquals(BioBuzzConstants.Auto.POLLEN_CAPACITY, g.pollenEnd());
        assertTrue("the intake still costs its time", g.step(3).durationSec > 0.0);
    }

    @Test
    public void theGiveUpPlanOfABlueRouteIsTheHalfTurnOfTheRedOne() {
        RouteAnalysis red = volleyThenLeave(20).build().analyze().giveUpPlan();
        RouteAnalysis blue = volleyThenLeave(20).build().forAlliance(Alliance.BLUE).analyze().giveUpPlan();
        assertEquals(red.totalSeconds(), blue.totalSeconds(), EPS);
        assertEquals(red.points(), blue.points());
        assertEquals(red.leave(), blue.leave());
        assertEquals(BioBuzzConstants.Field.SIZE_IN - red.endPose().x, blue.endPose().x, 1e-6);
    }

    // ---- the validator says when the safe points are lost --------------------------------------------------------------

    @Test
    public void aPartnerRouteThatEndsOnTheVolleyLosesItsLeaveWhenTheTipNeverComesAndTheValidatorSaysSo() {
        // no drive after the volley: the normal plan ends 14 in off the rear wall (LEAVE), but a robot that never fires stays touching it
        AutoRoute r = partnerRear().waitForTip(20).shootUp(REAR_X, REAR_Y, 90, 4).build();
        assertTrue("the normal plan earns LEAVE", r.analyze().leave());
        assertFalse("the give-up plan does not", r.analyze().giveUpPlan().leave());
        for (Alliance alliance : Alliance.values()) {
            List<Issue> l = RouteValidator.check(r.forAlliance(alliance));
            Issue found = null;
            for (Issue i : l) {
                if (i.rule == Rule.TIP_ASSUMPTION && i.message.contains("without LEAVE")) {
                    found = i;
                }
            }
            assertNotNull(alliance + ": " + l, found);
            assertEquals(Severity.WARN, found.severity);
            assertEquals("on the last step", 2, found.stepIndex);
            assertTrue(found.message, found.message.contains("if the tip these 1 volley depends on is never confirmed"));
            assertTrue(found.message, found.message.contains("the partner is late or never tips, or the camera is dead"));
            assertTrue(found.message, found.message.contains("no camera, no shot"));
            assertTrue(found.message, found.message.contains("without LEAVE (3 points)"));
            assertTrue(found.message, found.message.contains("Put the safe steps (the drive off the wall and the PARK) after the volley"));
        }
    }

    @Test
    public void aPartnerRouteWithTheSafeStepsAfterTheVolleyHasNothingToWarnAbout() {
        AutoRoute r = volleyThenLeave(20).build();
        for (Alliance alliance : Alliance.values()) {
            List<Issue> l = RouteValidator.check(r.forAlliance(alliance));
            for (Issue i : l) {
                assertFalse(alliance + ": " + i, i.message.contains("never confirmed"));
            }
        }
    }

    @Test
    public void aVolleyFiredFromInsideTheLoadingZoneLosesBothLeaveAndParkInTheGiveUpPlan() {
        // the normal plan ends at the volley's pose, 2.5 in off the red wall in the LOADING ZONE: LEAVE 3 + PARK 5; a robot that
        // leaves the volley out stays at its start on the rear wall and earns neither
        AutoRoute r = partnerRear().waitForTip(20).shootSide(Side.REAR, 10.5, 106.0, 0, 4).build();
        assertTrue(r.analyze().leave() && r.analyze().park());
        assertFalse(r.analyze().giveUpPlan().leave() || r.analyze().giveUpPlan().park());
        List<Issue> l = RouteValidator.check(r);
        Issue found = null;
        for (Issue i : l) {
            if (i.rule == Rule.TIP_ASSUMPTION && i.message.contains("never confirmed")) {
                found = i;
            }
        }
        assertNotNull(l.toString(), found);
        assertTrue(found.message, found.message.contains("without LEAVE and PARK (8 points)"));
    }

    // ---- the spill rule counts from the confirmed tip ------------------------------------------------------------------

    @Test
    public void theSpillZoneStaysForbiddenUntilTheCameraHasConfirmedTheTipPlusTheClearTime() {
        double clear = BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC;
        // stand right at the edge of the spill zone of the cell our volley tips (0.1 in short of it), wait, then step 1 in into it
        AutoRoute.Builder own = AutoRoute.builder("own").start(58, FRONT, -90).shootUp(58, AT_THE_ZONE_Y, -90, 4);
        // waiting exactly the clear time (3.0 s) after the tip leaves the step into the zone inside the confirmed window (tip + 0.5 + 3.0)
        List<Issue> inside = RouteValidator.check(own.waitSeconds(clear).driveConstant(58, AT_THE_ZONE_Y + 1.0, -90).build());
        assertEquals(inside.toString(), 1, count(inside, Rule.SPILL_G409, Severity.ERROR));
        // waiting the clear time plus the settle time is enough
        AutoRoute.Builder own2 = AutoRoute.builder("own2").start(58, FRONT, -90).shootUp(58, AT_THE_ZONE_Y, -90, 4);
        List<Issue> outside = RouteValidator.check(own2.waitSeconds(clear + SETTLE).driveConstant(58, AT_THE_ZONE_Y + 1.0, -90).build());
        assertEquals(outside.toString(), 0, count(outside, Rule.SPILL_G409, Severity.ERROR));
        // the message says when it is over
        Issue issue = null;
        for (Issue i : inside) {
            if (i.rule == Rule.SPILL_G409) {
                issue = i;
            }
        }
        assertTrue(issue.message, issue.message.contains("until its camera has confirmed the tip"));
        assertTrue(issue.message, issue.message.contains("stay out of the zone until"));
    }

    @Test
    public void theSettleTimeIsAddedToThePartnersSpillWindowToo() {
        double clear = BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC;
        // wait at the edge of the zone, then step 1 in at exactly tip + clear: inside the window that counts from the CONFIRMED tip
        AutoRoute.Builder base = AutoRoute.builder("partner").start(58, FRONT, -90).partnerTipsFirst().driveConstant(58, AT_THE_ZONE_Y, -90);
        double t0 = base.build().analyze().totalSeconds();
        assertTrue("the first drive is over well before the tip", t0 < TIP_SEC);
        List<Issue> inside = RouteValidator.check(AutoRoute.builder("partner").start(58, FRONT, -90).partnerTipsFirst().driveConstant(58, AT_THE_ZONE_Y, -90)
                .waitSeconds(TIP_SEC + clear - t0).driveConstant(58, AT_THE_ZONE_Y + 1.0, -90).build());
        assertEquals(inside.toString(), 1, count(inside, Rule.SPILL_G409, Severity.ERROR));
        assertTrue(inside.toString(), inside.toString().contains("the partner's assumed tip"));
        // one settle time later it is allowed
        List<Issue> outside = RouteValidator.check(AutoRoute.builder("partner").start(58, FRONT, -90).partnerTipsFirst().driveConstant(58, AT_THE_ZONE_Y, -90)
                .waitSeconds(TIP_SEC + clear + SETTLE - t0).driveConstant(58, AT_THE_ZONE_Y + 1.0, -90).build());
        assertEquals(outside.toString(), 0, count(outside, Rule.SPILL_G409, Severity.ERROR));
    }
}
