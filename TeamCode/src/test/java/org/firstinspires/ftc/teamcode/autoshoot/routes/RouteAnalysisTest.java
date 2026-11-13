package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.junit.Test;

/** The route simulation: effective poses, time, POLLEN, HIVE state and the points a route can claim. */
public class RouteAnalysisTest {
    private static final double EPS = 1e-9;

    /** C1 start, shoot the pre-loads, drive up the left corridor and park in the red LOADING ZONE. */
    static AutoRoute.Builder corridorPark() {
        return AutoRoute.builder("corridor_park").start(58, 9.2, -90)
                .shootUp(4)
                .driveReverseTangent(30, 30)
                .driveReverseTangent(30, 100)
                .driveLinear(12, 108, 0)
                .park(12, 108, 0);
    }

    @Test
    public void aShootThenParkRouteScoresLeaveParkAndTheFirstTip() {
        RouteAnalysis a = corridorPark().build().analyze();
        assertTrue(a.leave());
        assertTrue(a.park());
        // the 4 pre-loads on top of the 3 staged NECTAR are the first tip
        assertEquals(1, a.tips());
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.PARK_POINTS + BioBuzzConstants.Auto.TIP_POINTS,
                a.points());
        assertEquals(-4, a.pollenBalance());
        assertEquals(0, a.pollenEnd());
        assertEquals(4, a.pollenShot());
        assertEquals(0, a.pollenGained());
        assertTrue(a.parkOverlapIn() > 7.0);
    }

    @Test
    public void everyStepReportsTheEffectivePoseItLeavesTheRobotIn() {
        AutoRoute r = corridorPark().build();
        RouteAnalysis a = r.analyze();
        assertEquals(r.size(), a.steps().size());
        assertEquals(r.size(), r.endPoses().size());
        // the shoot step has no pose of its own: the robot is at the nominal standoff pose
        RouteAnalysis.StepInfo shoot = a.step(1);
        assertTrue(shoot.shootPoseIsNominal);
        assertEquals(59.25, shoot.to.x, 1e-6);
        assertEquals(-90.0, shoot.to.headingDegrees(), 1e-6);
        // reverse tangent up +y leaves the robot facing -y
        assertEquals(0.0, Math.abs(Angles.error(Math.toRadians(-90), a.step(3).to.heading)), 1e-6);
        // and the linear leg ends exactly at its target heading
        assertEquals(0.0, a.step(4).to.headingDegrees(), 1e-6);
        assertEquals(a.endPose().x, r.endPoses().get(r.size() - 1).x, EPS);
    }

    @Test
    public void timesAddUpAndEachStepStartsWhereThePreviousEnded() {
        RouteAnalysis a = corridorPark().build().analyze();
        double sum = 0;
        for (RouteAnalysis.StepInfo s : a.steps()) {
            assertEquals(sum, s.startSec, 1e-9);
            assertTrue(s.durationSec >= 0.0);
            sum += s.durationSec;
        }
        assertEquals(sum, a.totalSeconds(), 1e-9);
        assertEquals(0.0, a.step(0).durationSec, EPS);
        assertEquals("PARK is free: the drive before it paid the settle", 0.0, a.step(5).durationSec, EPS);
        assertTrue(a.totalSeconds() > 5 && a.totalSeconds() < 25);
    }

    // ---- the HIVE tip: staged NECTAR make the first tip one volley of 4 POLLEN --------------------------------

    @Test
    public void oneVolleyOfFourPollenTipsTheFirstHiveBecauseOfTheStagedNectar() {
        RouteAnalysis a = AutoRoute.builder("tip").start(35, 9.2, -90).shootUp(4).build().analyze();
        assertEquals(1, a.tips());
        assertEquals(BioBuzzConstants.Auto.TIP_POINTS, a.tipPoints());
        RouteAnalysis.StepInfo shoot = a.step(1);
        assertEquals(Side.AUDIENCE, shoot.upSideBefore);
        assertEquals("the cell that is UP at the start holds the staged NECTAR", BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL,
                shoot.nectarInCellBefore);
        assertTrue(shoot.tipped);
        assertEquals("the UP cell is on the other side after the tip", Side.REAR, shoot.upSideAfter);
        assertEquals("the counter starts again after a tip", 0, shoot.pollenInCellAfter);
        assertEquals("the last POLLEN of the volley is the one that tips", shoot.endSec(), shoot.tipSec, 1e-9);
        assertEquals(0, shoot.firedAfterTip);
    }

    @Test
    public void threePollenDoNotTipTheFirstHiveButFourDoEvenWhenTheyComeInTwoVolleys() {
        // 3 NECTAR + 3 POLLEN = 3 x 8 + 3 x 5 = 39, one short of 40: no tip
        RouteAnalysis three = AutoRoute.builder("three").start(35, 9.2, -90).shootUp(3).build().analyze();
        assertEquals(0, three.tips());
        assertFalse(three.step(1).tipped);
        assertEquals(3, three.step(1).pollenInCellAfter);
        assertEquals(Side.AUDIENCE, three.step(1).upSideAfter);
        assertTrue(Double.isNaN(three.step(1).tipSec));
        // the POLLEN of two volleys into the same cell add up: 3 + 1 = 4 tips
        RouteAnalysis split = AutoRoute.builder("split").start(35, 9.2, -90).shootUp(3).shootUp(1).build().analyze();
        assertEquals(1, split.tips());
        assertFalse(split.step(1).tipped);
        assertTrue(split.step(2).tipped);
        assertEquals(0, split.step(2).pollenInCellAfter);
    }

    @Test
    public void theSecondTipNeedsEightPollenBecauseTheNewUpCellHoldsNoNectar() {
        AutoRoute r = AutoRoute.builder("second").start(35, 9.2, -90)
                .shootUp(4)                                          // tip 1: 3 NECTAR + 4 POLLEN
                .intakeFlower(2.4, 48, 13.5, 48, 180, 4)
                .shootUp(4)                                          // 4 POLLEN into the empty REAR cell: not enough
                .intakeFlower(2.4, 48, 13.5, 48, 180, 4)
                .shootUp(4)                                          // 8 POLLEN in total: tip 2
                .build();
        RouteAnalysis a = r.analyze();
        assertTrue(a.step(1).tipped);
        RouteAnalysis.StepInfo half = a.step(3);
        assertEquals(Side.REAR, half.targetSide);
        assertEquals(0, half.nectarInCellBefore);
        assertFalse("4 POLLEN with no NECTAR are only half a tip", half.tipped);
        assertEquals(4, half.pollenInCellAfter);
        assertEquals(Side.REAR, half.upSideAfter);
        RouteAnalysis.StepInfo full = a.step(5);
        assertTrue(full.tipped);
        assertEquals(Side.REAR, full.targetSide);
        assertEquals(Side.AUDIENCE, full.upSideAfter);
        assertEquals(2, a.tips());
        assertEquals(2 * BioBuzzConstants.Auto.TIP_POINTS, a.tipPoints());
    }

    @Test
    public void aVolleyThatIsBiggerThanTheTipNeedsFiresItsLastPollenAfterTheTip() {
        // 2 POLLEN: 3 x 8 + 2 x 5 = 34, not yet. Then a volley of 4: the 2nd POLLEN of it makes 4 in the cell and tips.
        AutoRoute r = AutoRoute.builder("late").start(35, 9.2, -90)
                .shootUp(2).intakeFloor(40, 30, 38, 30, 0, 2).shootUp(4).build();
        RouteAnalysis.StepInfo volley = r.analyze().step(3);
        assertTrue(volley.tipped);
        assertEquals(2, volley.firedAfterTip);
        // the tip is half way through the volley's firing, not at its end
        double firing = volley.endSec() - volley.launchSec;
        assertEquals(volley.launchSec + firing * 2 / 4, volley.tipSec, 1e-9);
        assertTrue(volley.tipSec < volley.endSec());
    }

    @Test
    public void aVolleyAtTheDownCellDoesNotCountTowardTheTip() {
        // at match start the red REAR cell is DOWN: 4 POLLEN fired at it do nothing, and the UP cell stays AUDIENCE
        RouteAnalysis a = AutoRoute.builder("b").start(35, 9.2, -90).shootSide(Side.REAR, 4).build().analyze();
        assertEquals(0, a.tips());
        assertFalse(a.step(1).targetWasUp);
        assertEquals(Side.REAR, a.step(1).targetSide);
        assertEquals(Side.AUDIENCE, a.step(1).upSideAfter);
        // ...so a later volley at the UP cell still needs only 4 POLLEN
        RouteAnalysis later = AutoRoute.builder("b2").start(35, 9.2, -90).shootSide(Side.REAR, 2).shootUp(2).shootUp(2)
                .build().analyze();
        assertEquals(1, later.tips());
        assertTrue(later.step(3).tipped);
    }

    @Test
    public void afterATipTheNextVolleysCountInTheOtherCellFromZero() {
        AutoRoute r = AutoRoute.builder("c").start(35, 9.2, -90)
                .shootUp(4).shootUp(4).shootUp(3).build();
        RouteAnalysis a = r.analyze();
        assertEquals(1, a.tips());
        assertEquals(Side.REAR, a.step(2).targetSide);
        assertEquals(4, a.step(2).pollenInCellAfter);
        assertEquals("4 + 3 = 7 POLLEN are still one short of the 8 an empty cell needs", 7, a.step(3).pollenInCellAfter);
        assertFalse(a.step(3).tipped);
    }

    // ---- the partner shoots the first tip ----------------------------------------------------------------------

    private static final double PARTNER_SEC = BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC;

    /** A rear-wall start that only waits, so the HIVE state can be read off the clock. */
    private static AutoRoute.Builder partnerClock() {
        return AutoRoute.builder("partner_clock").start(61.5, 135.1, 90).partnerTipsFirst()
                .waitSeconds(5.0).waitSeconds(5.0).waitSeconds(5.0);
    }

    @Test
    public void thePartnerTipFlipsTheUpSideAtTheAssumedTimeAndNotBefore() {
        RouteAnalysis a = partnerClock().build().analyze();
        // waits end at 5, 10 and 15 s; the assumed tip is at 12 s
        assertEquals(Side.AUDIENCE, a.step(1).upSideAfter);
        assertEquals("still before the tip at 10 s", Side.AUDIENCE, a.step(2).upSideAfter);
        assertEquals(Side.AUDIENCE, a.step(3).upSideBefore);
        assertEquals("the third wait (10 to 15 s) crosses the tip", Side.REAR, a.step(3).upSideAfter);
        assertEquals(1, a.tipEvents().size());
        RouteAnalysis.TipEvent tip = a.tipEvents().get(0);
        assertTrue(tip.byPartner);
        assertEquals(PARTNER_SEC, tip.timeSec, EPS);
        assertEquals("the cell that was UP spills", Side.AUDIENCE, tip.releasingSide);
        assertEquals(Side.REAR, tip.newUpSide);
        assertEquals(-1, tip.stepIndex);
        assertTrue(a.partnerTipAssumed());
        assertEquals(PARTNER_SEC, a.partnerTipSec(), EPS);
        assertFalse(a.partnerTipVoided());
    }

    @Test
    public void aRouteThatDoesNotAssumeThePartnerNeverFlipsTheHiveByItself() {
        // the very same waits, but the route says SELF: nobody tips the HIVE, whatever the clock says
        RouteAnalysis a = AutoRoute.builder("self_clock").start(61.5, 135.1, 90)
                .waitSeconds(5.0).waitSeconds(5.0).waitSeconds(5.0).build().analyze();
        for (int i = 1; i < a.steps().size(); i++) {
            assertEquals(Side.AUDIENCE, a.step(i).upSideAfter);
        }
        assertTrue(a.tipEvents().isEmpty());
        assertFalse(a.partnerTipAssumed());
        assertTrue(Double.isNaN(a.partnerTipSec()));
        assertEquals(0, a.partnerTips());
        assertEquals(0, a.partnerTipPoints());
    }

    @Test
    public void thePartnerTipFlipsTheBlueHiveToItsAudienceSide() {
        // the blue HIVE starts with its REAR cell up (the half turn of red), so the partner's tip brings the AUDIENCE cell up
        RouteAnalysis a = partnerClock().build().forAlliance(Alliance.BLUE).analyze();
        assertEquals(Side.REAR, a.step(2).upSideAfter);
        assertEquals(Side.AUDIENCE, a.step(3).upSideAfter);
        assertEquals(Side.REAR, a.tipEvents().get(0).releasingSide);
        assertEquals(Side.AUDIENCE, a.tipEvents().get(0).newUpSide);
    }

    @Test
    public void aPartnerTipThatComesAfterTheRouteHasEndedStillHappens() {
        // the route is over at 5 s, the tip is assumed at 12 s: it is recorded all the same (the robot stands where it ended)
        RouteAnalysis a = AutoRoute.builder("short").start(61.5, 135.1, 90).partnerTipsFirst().waitSeconds(5.0).build().analyze();
        assertEquals(1, a.tipEvents().size());
        assertEquals(PARTNER_SEC, a.tipEvents().get(0).timeSec, EPS);
        assertEquals("the last step ended before the tip", Side.AUDIENCE, a.step(1).upSideAfter);
    }

    @Test
    public void waitForTipAdvancesTheClockToTheTipAndNoFurther() {
        RouteAnalysis a = AutoRoute.builder("wft").start(61.5, 135.1, 90).partnerTipsFirst()
                .waitSeconds(3.0).waitForTip(30.0).build().analyze();
        RouteAnalysis.StepInfo wait = a.step(2);
        assertEquals(3.0, wait.startSec, EPS);
        assertEquals("until the tip at 12 s", PARTNER_SEC - 3.0, wait.durationSec, EPS);
        assertEquals(PARTNER_SEC, wait.endSec(), EPS);
        assertEquals(PARTNER_SEC, wait.tipWaitSec, EPS);
        assertEquals(Side.AUDIENCE, wait.upSideBefore);
        assertEquals("the other cell is up when the wait ends", Side.REAR, wait.upSideAfter);
        assertEquals(PARTNER_SEC, a.totalSeconds(), EPS);
    }

    @Test
    public void waitForTipGivesUpAtItsTimeout() {
        RouteAnalysis a = AutoRoute.builder("wft").start(61.5, 135.1, 90).partnerTipsFirst().waitForTip(2.0).build().analyze();
        RouteAnalysis.StepInfo wait = a.step(1);
        assertEquals(2.0, wait.durationSec, EPS);
        assertEquals("the tip is still to come when the wait gives up", Side.AUDIENCE, wait.upSideAfter);
        assertEquals("it still knows when the tip is due", PARTNER_SEC, wait.tipWaitSec, EPS);
    }

    @Test
    public void waitForTipCostsNothingWhenThereIsNoTipToWaitFor() {
        // already tipped (the clock is at 15 s)
        RouteAnalysis late = partnerClock().waitForTip(20.0).build().analyze();
        RouteAnalysis.StepInfo done = late.step(4);
        assertEquals(0.0, done.durationSec, EPS);
        assertTrue(Double.isNaN(done.tipWaitSec));
        // a SELF route has no partner tip at all
        RouteAnalysis self = AutoRoute.builder("self").start(61.5, 135.1, 90).waitForTip(20.0).build().analyze();
        assertEquals(0.0, self.step(1).durationSec, EPS);
        assertTrue(Double.isNaN(self.step(1).tipWaitSec));
    }

    @Test
    public void thePartnersTipIsShownButNeverCountedInOurPoints() {
        RouteAnalysis a = partnerClock().build().analyze();
        assertEquals(1, a.partnerTips());
        assertEquals(BioBuzzConstants.Auto.TIP_POINTS, a.partnerTipPoints());
        assertEquals("our robot did not tip anything", 0, a.tips());
        assertEquals(0, a.tipPoints());
        // this route never leaves its start, which touches the rear wall: no LEAVE and no PARK either
        assertEquals(a.leavePoints() + a.parkPoints(), a.points());
        assertEquals(0, a.points());
        assertEquals(a.points(), partnerClock().build().claimablePoints());
    }

    @Test
    public void ourOwnTipBeforeTheAssumedTimeCancelsThePartnersTip() {
        // we shoot the 4 pre-loads at the audience cell at the start: that is the first tip, long before 12 s
        RouteAnalysis a = AutoRoute.builder("early").start(58, 9.2, -90).partnerTipsFirst().shootUp(4).waitSeconds(20).build()
                .analyze();
        assertEquals(1, a.tips());
        assertTrue(a.step(1).tipSec < PARTNER_SEC);
        assertTrue(a.partnerTipVoided());
        assertEquals("only our own tip is left", 1, a.tipEvents().size());
        assertFalse(a.tipEvents().get(0).byPartner);
        assertEquals(0, a.partnerTips());
        assertEquals(0, a.partnerTipPoints());
        assertEquals("the waiting later does not flip anything again", Side.REAR, a.step(2).upSideAfter);
    }

    @Test
    public void aVolleyThatIsStillFiringWhenThePartnerTipsDoesNotTipTheHiveItself() {
        // wait until 10.5 s, then fire 4 at the UP cell: the volley runs past 12 s, so the partner tips it first
        RouteAnalysis a = AutoRoute.builder("race").start(58, 9.2, -90).partnerTipsFirst().waitSeconds(10.5).shootUp(4).build()
                .analyze();
        RouteAnalysis.StepInfo shoot = a.step(2);
        assertTrue(shoot.launchSec < PARTNER_SEC && shoot.endSec() > PARTNER_SEC);
        assertTrue(shoot.targetWasUp);
        assertFalse("the partner's tip comes first, so ours does not happen", shoot.tipped);
        assertEquals(0, a.tips());
        assertEquals(1, a.partnerTips());
        assertFalse(a.partnerTipVoided());
        assertEquals(Side.REAR, shoot.upSideAfter);
    }

    @Test
    public void aShootThatStartsBeforeThePartnerTipTargetsTheCellThatIsUpThen() {
        // shootUp at 0 s: the UP cell is still the audience one; after a wait for the tip it is the rear one
        RouteAnalysis before = AutoRoute.builder("b").start(61.5, 135.1, 90).partnerTipsFirst().shootUp(2).build().analyze();
        assertEquals(Side.AUDIENCE, before.step(1).targetSide);
        RouteAnalysis after = AutoRoute.builder("a").start(61.5, 135.1, 90).partnerTipsFirst().waitForTip(30).shootUp(2).build()
                .analyze();
        assertEquals(Side.REAR, after.step(2).targetSide);
        assertTrue(after.step(2).targetWasUp);
        assertTrue("it launches at or after the tip", after.step(2).launchSec >= PARTNER_SEC - EPS);
    }

    @Test
    public void afterThePartnersTipOurVolleysNeedEightPollenBecauseTheRearCellHoldsNoNectar() {
        // wait for the tip, then 4 + 4 POLLEN into the rear cell: only the second volley tips it
        AutoRoute r = AutoRoute.builder("two").start(61.5, 135.1, 90).partnerTipsFirst()
                .waitForTip(30).shootUp(4)
                .intakeFlower(48, 141.6, 48, 133, 90, 4)
                .shootUp(4).build();
        RouteAnalysis a = r.analyze();
        assertFalse("4 POLLEN into the empty rear cell are not enough", a.step(2).tipped);
        assertEquals(0, a.step(2).nectarInCellBefore);
        assertTrue(a.step(4).tipped);
        assertEquals(1, a.tips());
        assertEquals(1, a.partnerTips());
        assertEquals("our points: only our own tip", BioBuzzConstants.Auto.TIP_POINTS, a.tipPoints());
        assertEquals(Side.AUDIENCE, a.step(4).upSideAfter);
    }

    // ---- where is the robot at a given time? -----------------------------------------------------------------

    @Test
    public void posesDuringADriveFollowTheSpeedProfileAndEndAtTheTargetOnceTheMoveIsOver() {
        // a straight 100 in leg along +y with the heading held at -90: speeds up at 60 in/s2 from rest
        RouteAnalysis a = AutoRoute.builder("drive").start(58, 9.2, -90).driveConstant(58, 109.2, -90).waitSeconds(5).build()
                .analyze();
        RouteAnalysis.StepInfo drive = a.step(1);
        // the first half second: the robot has gone 0.5 x 60 x 0.5^2 = 7.5 in
        List<Pose2> early = a.posesDuring(drive, 0.0, 0.5);
        assertEquals("starts where the leg starts", 9.2, early.get(0).y, 1e-9);
        assertEquals("has gone 7.5 in after 0.5 s", 9.2 + 7.5, early.get(early.size() - 1).y, 1e-6);
        double previous = 0;
        for (Pose2 p : early) {
            assertTrue("moves along +y only", p.y >= previous - 1e-9);
            previous = p.y;
        }
        // the whole step reaches the target and ends there
        List<Pose2> all = a.posesDuring(drive, 0.0, 100.0);
        assertEquals(109.2, all.get(all.size() - 1).y, 1e-9);
        // after the movement but before the settle time is over the robot stands at the target
        double travelled = drive.durationSec - BioBuzzConstants.Auto.SETTLE_SEC;
        List<Pose2> settling = a.posesDuring(drive, travelled + 0.01, drive.endSec());
        assertEquals(1, settling.size());
        assertEquals(109.2, settling.get(0).y, 1e-9);
        // during the next step (the wait) it stands still at the end of the drive
        List<Pose2> waiting = a.posesDuring(a.step(2), a.step(2).startSec + 1.0, a.step(2).startSec + 2.0);
        assertEquals(1, waiting.size());
        assertEquals(109.2, waiting.get(0).y, 1e-9);
    }

    @Test
    public void aStepThatIsNotRunningInTheWindowHasNoPoses() {
        RouteAnalysis a = AutoRoute.builder("drive").start(58, 9.2, -90).driveConstant(58, 109.2, -90).waitSeconds(5).build()
                .analyze();
        RouteAnalysis.StepInfo drive = a.step(1);
        assertTrue(a.posesDuring(drive, drive.endSec() + 1.0, drive.endSec() + 2.0).isEmpty());
        assertTrue(a.posesDuring(a.step(2), 0.0, 1.0).isEmpty());
    }

    @Test
    public void posesDuringATurnOnTheSpotIncludeTheTurningPoses() {
        // 90 degrees of turning in place at the start of a leg: the poses at the start point have different headings
        RouteAnalysis a = AutoRoute.builder("turn").start(58, 9.2, -90).driveConstant(58, 30, 0).build().analyze();
        List<Pose2> poses = a.posesDuring(a.step(1), 0.0, 0.0);
        boolean sawOtherHeading = false;
        for (Pose2 p : poses) {
            sawOtherHeading |= Math.abs(Angles.error(p.heading, Math.toRadians(-90))) > 0.1;
        }
        assertTrue("the robot turns before it moves", sawOtherHeading);
    }

    @Test
    public void theBlueHiveStartsWithItsRearCellUp() {
        AutoRoute blue = AutoRoute.builder("d").start(58, 9.2, -90).shootUp(4).build().forAlliance(Alliance.BLUE);
        RouteAnalysis a = blue.analyze();
        assertEquals(Side.REAR, a.step(1).upSideBefore);
        assertEquals(Side.REAR, a.step(1).targetSide);
        // the blue robot starts in the half-turn of C1 and shoots from the half-turn of the red nominal pose
        assertEquals(144 - 59.25, a.step(1).to.x, 1e-6);
        assertEquals(90.0, a.step(1).to.headingDegrees(), 1e-6);
    }

    @Test
    public void leaveNeedsTheWholeRobotOffEveryWallParkNeedsTheZone() {
        AutoRoute stay = AutoRoute.builder("stay").start(35, 9.2, -90).waitSeconds(1).build();
        assertFalse(stay.analyze().leave());
        assertEquals(0, stay.claimablePoints());

        AutoRoute off = AutoRoute.builder("off").start(35, 9.2, -90).driveConstant(35, 30, -90).build();
        assertTrue(off.analyze().leave());
        assertFalse(off.analyze().park());
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS, off.claimablePoints());

        // 1.0 in off the wall is inside the 1.5 in margin: not LEAVE
        AutoRoute near = AutoRoute.builder("near").start(35, 9.2, -90).driveConstant(35, BioBuzzConstants.Robot.LENGTH_IN / 2 + 1.0, -90)
                .build();
        assertFalse(near.analyze().leave());
    }

    @Test
    public void parkComesFromTheEndPoseEvenWithoutAParkStep() {
        AutoRoute r = AutoRoute.builder("e").start(8.9, 61.5, 180)
                .driveLinear(24.86, 83.6, 135).driveLinear(16, 91.5, 135).build();
        assertTrue(r.analyze().park());
        assertTrue(r.analyze().leave());
        assertEquals(8, r.claimablePoints());
    }

    @Test
    public void sourcesRunOutAfterFourPollen() {
        AutoRoute r = AutoRoute.builder("f").start(35, 9.2, -90)
                .shootUp(4)
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4)
                .shootUp(4)
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4)
                .build();
        RouteAnalysis a = r.analyze();
        assertEquals("own GARDEN", a.step(2).sourceName);
        assertEquals(4, a.step(2).sourceRemainingBefore);
        assertEquals(0, a.step(4).sourceRemainingBefore);
        assertFalse(a.step(2).sourceOffLimits);
    }

    @Test
    public void intakesRecogniseFlowersAndTheOtherAlliancesSources() {
        AutoRoute r = AutoRoute.builder("g").start(35, 9.2, -90)
                .intakeFlower(2.4, 48, 13.5, 48, 180, 4)
                .intakeFlower(96, 2.4, 70, 9, 0, 1)
                .intakeGarden(135, 138, 70, 9, 0, 1)
                .intakeFloor(40, 30, 38, 30, 0, 1)
                .build();
        RouteAnalysis a = r.analyze();
        assertEquals("FLOWER 1", a.step(1).sourceName);
        assertFalse(a.step(1).sourceOffLimits);
        assertEquals("FLOWER 4", a.step(2).sourceName);
        assertTrue("the audience-wall FLOWER at x = 96 is blue's", a.step(2).sourceOffLimits);
        assertEquals("other GARDEN", a.step(3).sourceName);
        assertTrue(a.step(3).sourceOffLimits);
        assertEquals("FLOOR", a.step(4).sourceName);
        assertNull(a.step(0).sourceName);
    }

    @Test
    public void redAndBlueAnalysesAgreeOnEverythingNotAboutCoordinates() {
        AutoRoute red = AutoRoute.builder("h").start(35, 9.2, -90)
                .shootUp(4).intakeGarden(5.9, 1.4, 9, 9.7, -90, 4).shootUp(4)
                .driveReverseTangent(30, 30).driveReverseTangent(30, 100).driveLinear(12, 108, 0).park(12, 108, 0)
                .build();
        RouteAnalysis a = red.analyze();
        RouteAnalysis b = red.forAlliance(Alliance.BLUE).analyze();
        assertEquals(a.totalSeconds(), b.totalSeconds(), 1e-9);
        assertEquals(a.points(), b.points());
        assertEquals(a.tips(), b.tips());
        assertEquals(a.pollenBalance(), b.pollenBalance());
        assertEquals(a.leave(), b.leave());
        assertEquals(a.park(), b.park());
        Pose2 ea = a.endPose();
        Pose2 eb = b.endPose();
        assertEquals(144 - ea.x, eb.x, 1e-6);
        assertEquals(144 - ea.y, eb.y, 1e-6);
        assertEquals(Math.PI, Math.abs(Angles.error(ea.heading, eb.heading)), 1e-6);
    }

    @Test
    public void routeSummaryMethodsDelegateToTheAnalysis() {
        AutoRoute r = corridorPark().build();
        assertEquals(r.analyze().totalSeconds(), r.estimatedSeconds(), EPS);
        assertEquals(r.analyze().points(), r.claimablePoints());
        assertEquals(r.analyze().pollenBalance(), r.pollenBalance());
    }
}
