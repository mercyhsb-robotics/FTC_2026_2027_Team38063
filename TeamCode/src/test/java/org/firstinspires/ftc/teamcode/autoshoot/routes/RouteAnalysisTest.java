package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.junit.Test;

/** The route simulation: effective poses, time, POLLEN, HIVE state and the points a route can claim. */
public class RouteAnalysisTest {
    /** The robot's center of rotation is this far (9.77 in) from a wall its FRONT edge is touching; see RobotShape. */
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double EPS = 1e-9;
    /** How long the camera must see a tipped cell steady before the robot calls the HIVE fully tipped. */
    private static final double SETTLE = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;

    /** C1 start, shoot the pre-loads, drive up the left corridor and park in the red LOADING ZONE. */
    static AutoRoute.Builder corridorPark() {
        return AutoRoute.builder("corridor_park").start(58, FRONT, -90)
                .shootUp(4)
                .driveConstant(30, 14.8, -90)    // slides sideways along the wall, staying low (the rear edge 47.8 in out is clear of the spill
                                                 // zone of the tipped cell, which reaches 40 in) with the front toward the wall: at the nominal
                                                 // shooting pose the center is only 12.1 in from the wall, too close to turn on the spot there
                .driveReverseTangent(30, 100)
                .driveLinear(12, 108, 0)
                .park(12, 108, 0);
    }

    @Test
    public void aShootThenParkRouteScoresLeaveParkAndTheFirstTip() {
        RouteAnalysis a = corridorPark().build().analyze();
        assertTrue(a.leave());
        assertTrue(a.park());
        // the 3rd of the 4 pre-loads, on top of the 3 staged NECTAR, is the first tip; the camera stops the feeder only after the 4th has left too
        assertEquals(1, a.tips());
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.PARK_POINTS + BioBuzzConstants.Auto.TIP_POINTS,
                a.points());
        assertEquals(-4, a.pollenBalance());
        assertEquals("the spare pre-load was launched into the swinging cell: the hopper is empty", 0, a.pollenEnd());
        assertEquals(4, a.pollenShot());
        assertEquals(0, a.pollenGained());
        assertTrue(a.parkOverlapIn() > 7.0);
        // a feeder that stopped in time would keep it
        RouteAnalysis fast = FastFeederRobot.analyze(corridorPark().build());
        assertEquals(1, fast.pollenEnd());
        assertEquals(3, fast.pollenShot());
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
        assertEquals(BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, shoot.to.x, 1e-6);
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

    // ---- the HIVE tip: the staged NECTAR make the first tip need only 3 POLLEN (FIRST's calibration) -------------

    @Test
    public void aVolleyOfFourTipsTheFirstHiveOnItsThirdPollenAndTheFourthIsLaunchedBeforeTheFeederCanStop() {
        RouteAnalysis a = AutoRoute.builder("tip").start(35, FRONT, -90).shootUp(4).build().analyze();
        assertEquals(1, a.tips());
        assertEquals(BioBuzzConstants.Auto.TIP_POINTS, a.tipPoints());
        RouteAnalysis.StepInfo shoot = a.step(1);
        assertEquals(Side.AUDIENCE, shoot.upSideBefore);
        assertEquals("the cell that is UP at the start holds the staged NECTAR", BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL,
                shoot.nectarInCellBefore);
        assertTrue(shoot.tipped);
        assertEquals("the UP cell is on the other side after the tip", Side.REAR, shoot.upSideAfter);
        assertEquals("the counter starts again after a tip", 0, shoot.pollenInCellAfter);
        // the tip is when the 3rd POLLEN LANDS: after the aim and spin-up, three feeds, and the flight
        assertEquals("the tip comes after the aim and spin-up, three feeds and the flight of the 3rd POLLEN", shoot.launchSec
                + BioBuzzConstants.Auto.AIM_SPINUP_SEC + 3 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC + BioBuzzConstants.Auto.BALL_FLIGHT_SEC, shoot.tipSec, 1e-9);
        assertEquals("3 POLLEN tip the cell that holds the staged NECTAR", 3, shoot.tipNeeded);
        assertEquals("one pre-load is planned as the spare", 1, shoot.tipSpare);
        // the 4th leaves 1.0 s after the 3rd, the camera stops the feeder 0.85 s after the 3rd LANDS (0.5 s after it left): 1.35 s after it left
        assertEquals("the 4th POLLEN is launched into the cell that is swinging down", 1, shoot.firedAfterTip);
        assertEquals("all 4 left the robot", 4, shoot.pollenBefore - shoot.pollenAfter);
        assertEquals("nothing is left as a spare", 0, shoot.pollenAfter);
        assertEquals("the volley ends when the 4th has left", shoot.launchSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC
                + 4 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC, shoot.endSec(), 1e-9);
        assertEquals("the plan still had a spare of 1", 1, a.minimumTipSpare());
    }

    /**
     * Situation: the same volley of 4 for a robot whose feeder stops the instant the camera sees the tip ({@link FastFeederRobot}).
     * Expected: the volley fires only the 3 that tip, keeps the 4th, ends when the 3rd has left (the 3rd is then still in the air) and the tip lands
     *     one flight time later. This is the old model, kept for the tests about other rules.
     */
    @Test
    public void aFeederThatStopsInTimeKeepsTheFourthPollenAsTheSpare() {
        RouteAnalysis a = FastFeederRobot.analyze(AutoRoute.builder("tip").start(35, FRONT, -90).shootUp(4).build());
        RouteAnalysis.StepInfo shoot = a.step(1);
        assertEquals(0, shoot.firedAfterTip);
        assertEquals(3, shoot.pollenBefore - shoot.pollenAfter);
        assertEquals(1, shoot.pollenAfter);
        assertEquals("the volley ends when the 3rd has left", shoot.launchSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC
                + 3 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC, shoot.endSec(), 1e-9);
        assertEquals("and the tip lands a flight time later", shoot.endSec() + BioBuzzConstants.Auto.BALL_FLIGHT_SEC, shoot.tipSec, 1e-9);
    }

    @Test
    public void threePollenTipTheFirstHiveAndTwoDoNotEvenWhenTheyComeInTwoVolleys() {
        // 3 NECTAR + 3 POLLEN = 3 x 5 + 3 x 3 = 24 thirds, the 24 that tip a cell (FIRST: the 3rd POLLEN tips it)
        RouteAnalysis three = AutoRoute.builder("three").start(35, FRONT, -90).shootUp(3).build().analyze();
        assertEquals(1, three.tips());
        assertTrue(three.step(1).tipped);
        assertEquals("3 of 3: exactly the minimum, no spare", 0, three.step(1).tipSpare);
        assertEquals(0, three.minimumTipSpare());
        assertEquals(Side.REAR, three.step(1).upSideAfter);
        // 3 NECTAR + 2 POLLEN = 21 thirds, one POLLEN short: no tip (the case the old weighted rule got wrong the other way round)
        RouteAnalysis two = AutoRoute.builder("two").start(35, FRONT, -90).shootUp(2).build().analyze();
        assertEquals(0, two.tips());
        assertFalse(two.step(1).tipped);
        assertEquals(2, two.step(1).pollenInCellAfter);
        assertEquals(Side.AUDIENCE, two.step(1).upSideAfter);
        assertTrue(Double.isNaN(two.step(1).tipSec));
        assertEquals("no tip, so no spare to report", -1, two.minimumTipSpare());
        // the POLLEN of two volleys into the same cell add up: 2 + 1 = 3 tips
        RouteAnalysis split = AutoRoute.builder("split").start(35, FRONT, -90).shootUp(2).shootUp(1).build().analyze();
        assertEquals(1, split.tips());
        assertFalse(split.step(1).tipped);
        assertTrue(split.step(2).tipped);
        assertEquals(1, split.step(2).tipNeeded);
        assertEquals(0, split.step(2).pollenInCellAfter);
    }

    @Test
    public void theSecondTipNeedsEightPollenBecauseTheNewUpCellHoldsNoNectar() {
        AutoRoute r = AutoRoute.builder("second").start(35, FRONT, -90)
                .shootUp(4)                                          // tip 1: 3 NECTAR + the 3rd POLLEN, the 4th stays in the robot
                .intakeFlower(2.4, 48, 3)             // 1 + 3 = 4 in the robot
                .shootUp(4)                                          // 4 POLLEN into the empty REAR cell: not enough
                .intakeFlower(2.4, 48, 4)
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
        assertEquals("the second tip needed the 4 of this volley on top of the 4 already in the cell", 4, full.tipNeeded);
        assertEquals("8 POLLEN is exactly the minimum for an empty cell: no spare", 0, full.tipSpare);
        assertEquals("the first tip has one spare, the second none: the route is as safe as its tightest tip", 0, a.minimumTipSpare());
    }

    @Test
    public void nineIntoAnEmptyCellLeavesOneSpare() {
        // 4 + 3 + 2: the 8th POLLEN tips an empty cell, the 9th is the spare for a field that is one POLLEN stiffer
        RouteAnalysis a = AutoRoute.builder("nine").start(35, FRONT, -90)
                .shootUp(4).shootSide(Side.REAR, 4).shootSide(Side.REAR, 3).shootSide(Side.REAR, 2).build().analyze();
        assertTrue("the first tip", a.step(1).tipped);
        assertFalse("4 POLLEN in the empty rear cell", a.step(2).tipped);
        assertFalse("7 POLLEN: one short", a.step(3).tipped);
        assertTrue("the 8th POLLEN tips it", a.step(4).tipped);
        assertEquals(1, a.step(4).tipNeeded);
        assertEquals("9 planned into the cell: 1 spare", 1, a.step(4).tipSpare);
        assertEquals(2, a.tips());
    }

    @Test
    public void aCameraGatedVolleyThatIsBiggerThanTheTipNeedsStopsOnlyWhenTheCameraHasSeenTheTip() {
        // 2 POLLEN: 3 NECTAR + 2 POLLEN = 21 thirds, not yet. Then a volley of 4: the 1st POLLEN of it makes 3 in the cell and tips.
        AutoRoute r = AutoRoute.builder("late").start(35, FRONT, -90)
                .shootUp(2).intakeFloor(40, 30, 38, 30, 0, 2).shootUp(4).build();
        RouteAnalysis.StepInfo volley = r.analyze().step(3);
        assertTrue(volley.tipped);
        assertEquals(1, volley.tipNeeded);
        assertEquals("3 of the 4 are the spare", 3, volley.tipSpare);
        // the 1st POLLEN lands 0.5 s after it left; the feeder stops 0.85 s after that, 1.35 s after the 1st left: the 2nd (1.0 s after the 1st) is launched,
        // the 3rd (2.0 s) is not
        assertEquals("the 2nd POLLEN is launched into the tipping cell, the 3rd and 4th are not", 1, volley.firedAfterTip);
        assertEquals("two POLLEN left the robot, two stayed", 2, volley.pollenBefore - volley.pollenAfter);
        assertEquals(volley.launchSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC + BioBuzzConstants.Auto.TIME_PER_SHOT_SEC
                + BioBuzzConstants.Auto.BALL_FLIGHT_SEC, volley.tipSec, 1e-9);
        assertEquals("the volley ends when the 2nd has left", volley.launchSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC
                + 2 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC, volley.endSec(), 1e-9);
        // with a feeder that stops in time: nothing is launched into the tipping cell
        RouteAnalysis.StepInfo fast = FastFeederRobot.analyze(r).step(3);
        assertEquals(0, fast.firedAfterTip);
        assertEquals(1, fast.pollenBefore - fast.pollenAfter);
        assertEquals(fast.tipSec - BioBuzzConstants.Auto.BALL_FLIGHT_SEC, fast.endSec(), 1e-9);
    }

    @Test
    public void aTimedVolleyCannotStopAndLaunchesItsLastPollenAfterTheTip() {
        // an old OpMode feeds for a fixed 8 s whatever happens: the 3rd of its 4 POLLEN tips the HIVE and the 4th goes into a cell
        // that is already tipping down
        AutoRoute r = AutoRoute.builder("timed").start(35, FRONT, -90).shootUp(35, FRONT, -90, 4).launchSeconds(8.0).build();
        RouteAnalysis.StepInfo volley = r.analyze().step(1);
        assertTrue(volley.tipped);
        assertEquals(3, volley.tipNeeded);
        assertEquals(1, volley.tipSpare);
        assertEquals("the 4th is launched after the tip", 1, volley.firedAfterTip);
        assertEquals("all 4 left the robot", 4, volley.pollenBefore - volley.pollenAfter);
        // the POLLEN leave at an even pace: the 3rd of 4 leaves three quarters of the way through the 8 s, and lands a flight time later
        assertEquals(volley.launchSec + 8.0 * 3 / 4 + BioBuzzConstants.Auto.BALL_FLIGHT_SEC, volley.tipSec, 1e-9);
        assertEquals("the volley lasts its fixed 8 s", volley.launchSec + 8.0, volley.endSec(), 1e-9);
    }

    @Test
    public void aVolleyAtTheDownCellDoesNotCountTowardTheTip() {
        // at match start the red REAR cell is DOWN: 4 POLLEN fired at it do nothing, and the UP cell stays AUDIENCE
        RouteAnalysis a = AutoRoute.builder("b").start(35, FRONT, -90).shootSide(Side.REAR, 4).build().analyze();
        assertEquals(0, a.tips());
        assertFalse(a.step(1).targetWasUp);
        assertEquals(Side.REAR, a.step(1).targetSide);
        assertEquals(Side.AUDIENCE, a.step(1).upSideAfter);
        // ...so a later volley at the UP cell still needs only 4 POLLEN
        RouteAnalysis later = AutoRoute.builder("b2").start(35, FRONT, -90).shootSide(Side.REAR, 2).shootUp(2).shootUp(2)
                .build().analyze();
        assertEquals(1, later.tips());
        assertTrue(later.step(3).tipped);
    }

    @Test
    public void afterATipTheNextVolleysCountInTheOtherCellFromZero() {
        AutoRoute r = AutoRoute.builder("c").start(35, FRONT, -90)
                .shootUp(4).shootUp(4).shootUp(3).build();
        RouteAnalysis a = r.analyze();
        assertEquals(1, a.tips());
        assertEquals(Side.REAR, a.step(2).targetSide);
        assertEquals(4, a.step(2).pollenInCellAfter);
        assertEquals("4 + 3 = 7 POLLEN are still one short of the 8 an empty cell needs", 7, a.step(3).pollenInCellAfter);
        assertFalse(a.step(3).tipped);
    }

    @Test
    public void eachVolleyRemembersWhatItsCellHeldBeforeItStarted() {
        // 2 POLLEN, then 1 more at the same cell: the second volley starts with 2 POLLEN of ours already in it
        RouteAnalysis split = AutoRoute.builder("split").start(35, FRONT, -90).shootUp(2).shootUp(1).build().analyze();
        assertEquals(0, split.step(1).pollenInCellBefore);
        assertEquals(3, split.step(1).nectarInCellBefore);
        assertEquals(2, split.step(2).pollenInCellBefore);
        assertEquals(3, split.step(2).nectarInCellBefore);
        // after the tip the other cell starts empty: no NECTAR and no POLLEN of ours
        RouteAnalysis tipped = AutoRoute.builder("tipped").start(35, FRONT, -90).shootUp(4).shootUp(2).build().analyze();
        assertEquals(0, tipped.step(2).pollenInCellBefore);
        assertEquals(0, tipped.step(2).nectarInCellBefore);
        // a volley at the cell that faces DOWN has nothing in it that counts
        RouteAnalysis down = AutoRoute.builder("down").start(35, FRONT, -90).shootSide(Side.REAR, 2).build().analyze();
        assertEquals(0, down.step(1).pollenInCellBefore);
        assertEquals(0, down.step(1).nectarInCellBefore);
    }

    // ---- the partner shoots the first tip ----------------------------------------------------------------------

    private static final double PARTNER_SEC = BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC;

    /** A rear-wall start that only waits, so the HIVE state can be read off the clock. */
    private static AutoRoute.Builder partnerClock() {
        return AutoRoute.builder("partner_clock").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst()
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
        RouteAnalysis a = AutoRoute.builder("self_clock").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90)
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
        RouteAnalysis a = AutoRoute.builder("short").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitSeconds(5.0).build().analyze();
        assertEquals(1, a.tipEvents().size());
        assertEquals(PARTNER_SEC, a.tipEvents().get(0).timeSec, EPS);
        assertEquals("the last step ended before the tip", Side.AUDIENCE, a.step(1).upSideAfter);
    }

    @Test
    public void waitForTipAdvancesTheClockToTheTipAndNoFurther() {
        RouteAnalysis a = AutoRoute.builder("wft").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst()
                .waitSeconds(3.0).waitForTip(30.0).build().analyze();
        RouteAnalysis.StepInfo wait = a.step(2);
        assertEquals(3.0, wait.startSec, EPS);
        assertEquals("until the tip at 12 s has been seen steady for the settle time", PARTNER_SEC + SETTLE - 3.0, wait.durationSec, EPS);
        assertEquals(PARTNER_SEC + SETTLE, wait.endSec(), EPS);
        assertEquals("the step still knows when the tip itself is due", PARTNER_SEC, wait.tipWaitSec, EPS);
        assertEquals(Side.AUDIENCE, wait.upSideBefore);
        assertEquals("the other cell is up when the wait ends", Side.REAR, wait.upSideAfter);
        assertEquals(PARTNER_SEC + SETTLE, a.totalSeconds(), EPS);
        assertEquals(PARTNER_SEC + SETTLE, a.partnerTipConfirmedSec(), EPS);
        assertEquals(SETTLE, a.tipSettleSec(), EPS);
    }

    @Test
    public void theSettleTimeIsRequiredBeforeTheRobotGoesOn() {
        // Without the settle time the robot would go on at the very moment of the tip, while the HIVE is still rocking (a TIP is
        // complete only when the damper touches the frame, and launching at a rocking HIVE can spoil it, G417).
        assertTrue("the settle time is a real, positive time", SETTLE > 0.0);
        RouteAnalysis a = AutoRoute.builder("wft").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst()
                .waitForTip(30.0).build().analyze();
        assertTrue("the wait does not end at the tip", a.step(1).endSec() > PARTNER_SEC + EPS);
        assertEquals(PARTNER_SEC + SETTLE, a.step(1).endSec(), EPS);
        // the same settle time is the constant the camera tracker uses for "seen steadily"
        assertEquals(BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC, a.tipSettleSec(), 0.0);
        // a self route has no partner tip to confirm
        assertTrue(Double.isNaN(AutoRoute.builder("self").start(58, FRONT, -90).shootUp(4).build().analyze().partnerTipConfirmedSec()));
    }

    @Test
    public void waitForTipGivesUpAtItsDeadlineOnTheRouteClock() {
        RouteAnalysis a = AutoRoute.builder("wft").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitForTip(2.0).build().analyze();
        RouteAnalysis.StepInfo wait = a.step(1);
        assertEquals(2.0, wait.durationSec, EPS);
        assertEquals("the tip is still to come when the wait gives up", Side.AUDIENCE, wait.upSideAfter);
        assertEquals("it still knows when the tip is due", PARTNER_SEC, wait.tipWaitSec, EPS);
        // the number is a time ON THE CLOCK, not a length: after 3 s of other waiting, "give up at 5" is only 2 s more
        RouteAnalysis later = AutoRoute.builder("wft2").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitSeconds(3.0).waitForTip(5.0)
                .build().analyze();
        assertEquals(3.0, later.step(2).startSec, EPS);
        assertEquals(2.0, later.step(2).durationSec, EPS);
        assertEquals(5.0, later.step(2).endSec(), EPS);
        // a deadline that has passed already means no waiting at all
        RouteAnalysis passed = AutoRoute.builder("wft3").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitSeconds(8.0).waitForTip(5.0)
                .build().analyze();
        assertEquals(0.0, passed.step(2).durationSec, EPS);
    }

    // ---- how late may the partner be? ---------------------------------------------------------------------------

    private static final double BUDGET = BioBuzzConstants.Auto.AUTO_DURATION_SEC - BioBuzzConstants.Auto.TIME_SAFETY_MARGIN_SEC;

    /** Wait for the tip (give up at the given time) and fire 4 POLLEN from a pose beside the start: about 6 s of work. */
    private static AutoRoute.Builder waitThenFire(double giveUpSec) {
        return AutoRoute.builder("wf").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitForTip(giveUpSec).shootUp(61.5, 130.0, 90, 4);
    }

    @Test
    public void theBudgetIsTheAutoLimitMinusTheSafetyMargin() {
        assertEquals(28.0, RouteAnalysis.timeBudgetSec(), EPS);
    }

    @Test
    public void theLatestGiveUpTimeIsTheBudgetMinusWhatTheRestOfTheRouteTakes() {
        RouteAnalysis a = waitThenFire(20.0).build().analyze();
        RouteAnalysis.StepInfo wait = a.step(1);
        double rest = a.totalSeconds() - wait.endSec();
        assertTrue("the volley takes a few seconds: " + rest, rest > 5.0 && rest < 8.0);
        assertEquals(BUDGET - rest, a.latestGiveUpSec(wait), EPS);
        assertTrue("only a WAIT_FOR_TIP has one", Double.isNaN(a.latestGiveUpSec(a.step(2))));
        // a longer route after the wait leaves less time to wait
        RouteAnalysis longer = waitThenFire(20.0).waitSeconds(4.0).build().analyze();
        assertEquals(a.latestGiveUpSec(wait) - 4.0, longer.latestGiveUpSec(longer.step(1)), EPS);
    }

    @Test
    public void theLatestPartnerTipIsTheGiveUpTimeWhenThatComesFirst() {
        RouteAnalysis a = waitThenFire(18.0).build().analyze();
        assertEquals("the wait gives up at 18 s, and the camera needs the settle time after the tip", 18.0 - SETTLE, a.latestPartnerTipSec(), EPS);
        assertEquals("slack = latest minus assumed", 18.0 - SETTLE - PARTNER_SEC, a.partnerTipSlackSec(), EPS);
        assertEquals(PARTNER_SEC, a.partnerTipSec(), EPS);
    }

    @Test
    public void theLatestPartnerTipIsTheTimeBudgetWhenTheGiveUpTimeIsLater() {
        // giving up at 28 s is later than the rest of the route allows: the budget is the limit
        RouteAnalysis a = waitThenFire(28.0).build().analyze();
        double expected = a.latestGiveUpSec(a.step(1)) - SETTLE;
        assertTrue(expected < 28.0);
        assertEquals(expected, a.latestPartnerTipSec(), EPS);
        assertEquals(expected - PARTNER_SEC, a.partnerTipSlackSec(), EPS);
    }

    @Test
    public void theLatestPartnerTipIsReallyTheLastOneForWhichTheWholeRouteStillWorks() {
        // Check the formula against the simulation itself: put the partner's tip at the latest time, then 0.3 s later.
        for (double giveUp : new double[] {15.0, 18.0, 28.0}) {
            AutoRoute route = waitThenFire(giveUp).build();
            double latest = route.analyze().latestPartnerTipSec();
            // at the latest time: the wait ends at the tip, the volley follows it and the route ends inside the budget
            RouteAnalysis atLatest = RouteAnalysis.ofWithPartnerTipAt(route, latest);
            assertTrue("giveUp " + giveUp, atLatest.totalSeconds() <= BUDGET + 1e-6);
            RouteAnalysis.StepInfo shoot = atLatest.step(2);
            assertTrue(shoot.targetWasUp);
            assertTrue("the volley starts at or after the CONFIRMED tip", shoot.launchSec >= latest + SETTLE - 1e-6);
            // 0.3 s later it does not: the robot gives up waiting (the volley would go into a cell that faces down), or the route
            // runs past the budget
            RouteAnalysis tooLate = RouteAnalysis.ofWithPartnerTipAt(route, latest + 0.3);
            boolean gaveUp = tooLate.step(1).endSec() < latest + 0.3 + SETTLE - 1e-9;
            boolean overran = tooLate.totalSeconds() > BUDGET + 1e-6;
            assertTrue("giveUp " + giveUp + ": " + tooLate.totalSeconds(), gaveUp || overran);
        }
    }

    @Test
    public void aTipEarlierThanAssumedNeverCostsTimeBeyondTheWait() {
        // the earliest possible partner tip (before the route starts waiting) just removes the wait
        AutoRoute route = waitThenFire(20.0).build();
        RouteAnalysis early = RouteAnalysis.ofWithPartnerTipAt(route, 0.5);
        RouteAnalysis assumed = route.analyze();
        assertTrue(early.totalSeconds() < assumed.totalSeconds());
        assertEquals("the wait ends when the early tip has been seen steady", 0.5 + SETTLE, early.step(1).endSec(), EPS);
        assertEquals("the volley itself takes as long as before", assumed.step(2).durationSec, early.step(2).durationSec, EPS);
    }

    @Test
    public void aRouteThatDoesNotDependOnThePartnersTipHasNoLatestTime() {
        // it never shoots at the cell that comes up and never waits for it: any partner time is fine
        RouteAnalysis walk = AutoRoute.builder("walk").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().driveConstant(61.5, 120, 90).build()
                .analyze();
        assertEquals(Double.POSITIVE_INFINITY, walk.latestPartnerTipSec(), 0.0);
        assertEquals(Double.POSITIVE_INFINITY, walk.partnerTipSlackSec(), 0.0);
        // our own tip came first: the partner is not needed any more
        RouteAnalysis own = AutoRoute.builder("own").start(58, FRONT, -90).partnerTipsFirst().shootUp(4).waitSeconds(5).build().analyze();
        assertTrue(own.partnerTipVoided());
        assertEquals(Double.POSITIVE_INFINITY, own.latestPartnerTipSec(), 0.0);
        // a route that shoots the first tip itself has no partner tip to be late
        RouteAnalysis self = AutoRoute.builder("self").start(58, FRONT, -90).shootUp(4).build().analyze();
        assertTrue(Double.isNaN(self.latestPartnerTipSec()));
        assertTrue(Double.isNaN(self.partnerTipSlackSec()));
    }

    @Test
    public void withoutAWaitTheLatestTipIsTheMomentTheVolleyStarts() {
        // (the checker calls such a route an ERROR; the number is still defined: the tip must be done when the volley starts)
        RouteAnalysis a = AutoRoute.builder("nowait").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitSeconds(13.0)
                .shootUp(61.5, 130.0, 90, 4).build().analyze();
        assertEquals("the tip must be confirmed (done plus the settle time) when the volley starts", a.step(2).launchSec - SETTLE,
                a.latestPartnerTipSec(), EPS);
    }

    @Test
    public void theLatestTipOfARouteWithALongTailIsBelowTheAssumedTimeAndTheSlackIsNegative() {
        // 4, fetch 4 from the rear FLOWER, 4 again: about 18 s of work after the wait, the assumed tip at 12 s is too late
        RouteAnalysis a = waitThenFire(20.0).intakeFlower(BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][1], IntakeWheel.LEFT, 4).shootUp(61.5, 130.0, 90, 4).build().analyze();
        assertTrue(a.latestPartnerTipSec() < PARTNER_SEC);
        assertTrue(a.partnerTipSlackSec() < 0.0);
    }

    @Test
    public void waitForTipCostsNothingWhenThereIsNoTipToWaitFor() {
        // already tipped (the clock is at 15 s)
        RouteAnalysis late = partnerClock().waitForTip(20.0).build().analyze();
        RouteAnalysis.StepInfo done = late.step(4);
        assertEquals(0.0, done.durationSec, EPS);
        assertTrue(Double.isNaN(done.tipWaitSec));
        // a SELF route has no partner tip at all
        RouteAnalysis self = AutoRoute.builder("self").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).waitForTip(20.0).build().analyze();
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
        RouteAnalysis a = AutoRoute.builder("early").start(58, FRONT, -90).partnerTipsFirst().shootUp(4).waitSeconds(20).build()
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
        RouteAnalysis a = AutoRoute.builder("race").start(58, FRONT, -90).partnerTipsFirst().waitSeconds(10.5).shootUp(4).build()
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
        RouteAnalysis before = AutoRoute.builder("b").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().shootUp(2).build().analyze();
        assertEquals(Side.AUDIENCE, before.step(1).targetSide);
        RouteAnalysis after = AutoRoute.builder("a").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitForTip(30).shootUp(2).build()
                .analyze();
        assertEquals(Side.REAR, after.step(2).targetSide);
        assertTrue(after.step(2).targetWasUp);
        assertTrue("it launches at or after the tip", after.step(2).launchSec >= PARTNER_SEC - EPS);
    }

    @Test
    public void afterThePartnersTipOurVolleysNeedEightPollenBecauseTheRearCellHoldsNoNectar() {
        // wait for the tip, then 4 + 4 POLLEN into the rear cell: only the second volley tips it
        AutoRoute r = AutoRoute.builder("two").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst()
                .waitForTip(30).shootUp(4)
                .intakeFlower(BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][1], IntakeWheel.LEFT, 4)
                .shootUp(4).build();
        RouteAnalysis a = r.analyze();
        assertFalse("4 POLLEN into the empty rear cell are not enough", a.step(2).tipped);
        assertEquals(0, a.step(2).nectarInCellBefore);
        assertTrue(a.step(4).tipped);
        assertEquals("4 + 4 into an empty cell is exactly the 8 that tip it: no spare", 0, a.step(4).tipSpare);
        assertEquals(1, a.tips());
        assertEquals(1, a.partnerTips());
        assertEquals("our points: only our own tip", BioBuzzConstants.Auto.TIP_POINTS, a.tipPoints());
        assertEquals(Side.AUDIENCE, a.step(4).upSideAfter);
    }

    // ---- where is the robot at a given time? -----------------------------------------------------------------

    @Test
    public void posesDuringADriveFollowTheSpeedProfileAndEndAtTheTargetOnceTheMoveIsOver() {
        // a straight 100 in leg along +y with the heading held at -90: speeds up at the acceleration of the constants from rest
        RouteAnalysis a = AutoRoute.builder("drive").start(58, 9.2, -90).driveConstant(58, 109.2, -90).waitSeconds(5).build()
                .analyze();
        RouteAnalysis.StepInfo drive = a.step(1);
        // half way through the speed-up (half of v / a seconds): the robot has gone 0.5 x a x t^2 inches
        double v = BioBuzzConstants.Auto.CRUISE_SPEED_IN_PER_SEC;
        double accel = BioBuzzConstants.Auto.ACCEL_IN_PER_SEC2;
        double halfRamp = 0.5 * v / accel;
        double gone = 0.5 * accel * halfRamp * halfRamp;
        List<Pose2> early = a.posesDuring(drive, 0.0, halfRamp);
        assertEquals("starts where the leg starts", 9.2, early.get(0).y, 1e-9);
        assertEquals("has gone " + gone + " in after " + halfRamp + " s", 9.2 + gone, early.get(early.size() - 1).y, 1e-6);
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
        RouteAnalysis a = AutoRoute.builder("turn").start(58, FRONT, -90).driveConstant(58, 30, 0).build().analyze();
        List<Pose2> poses = a.posesDuring(a.step(1), 0.0, 0.0);
        boolean sawOtherHeading = false;
        for (Pose2 p : poses) {
            sawOtherHeading |= Math.abs(Angles.error(p.heading, Math.toRadians(-90))) > 0.1;
        }
        assertTrue("the robot turns before it moves", sawOtherHeading);
    }

    @Test
    public void theBlueHiveStartsWithItsRearCellUp() {
        AutoRoute blue = AutoRoute.builder("d").start(58, FRONT, -90).shootUp(4).build().forAlliance(Alliance.BLUE);
        RouteAnalysis a = blue.analyze();
        assertEquals(Side.REAR, a.step(1).upSideBefore);
        assertEquals(Side.REAR, a.step(1).targetSide);
        // the blue robot starts in the half-turn of C1 and shoots from the half-turn of the red nominal pose
        assertEquals(BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, a.step(1).to.x, 1e-6);
        assertEquals(90.0, a.step(1).to.headingDegrees(), 1e-6);
    }

    @Test
    public void leaveNeedsTheWholeRobotOffEveryWallParkNeedsTheZone() {
        AutoRoute stay = AutoRoute.builder("stay").start(35, FRONT, -90).waitSeconds(1).build();
        assertFalse(stay.analyze().leave());
        assertEquals(0, stay.claimablePoints());

        AutoRoute off = AutoRoute.builder("off").start(35, FRONT, -90).driveConstant(35, 30, -90).build();
        assertTrue(off.analyze().leave());
        assertFalse(off.analyze().park());
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS, off.claimablePoints());

        // 1.0 in off the wall is inside the 1.5 in margin: not LEAVE
        AutoRoute near = AutoRoute.builder("near").start(35, FRONT, -90).driveConstant(35, FRONT + 1.0, -90)
                .build();
        assertFalse(near.analyze().leave());
    }

    @Test
    public void parkComesFromTheEndPoseEvenWithoutAParkStep() {
        AutoRoute r = AutoRoute.builder("e").start(FRONT, 61.5, 180)
                .driveLinear(24.86, 83.6, 135).driveLinear(16, 91.5, 135).build();
        assertTrue(r.analyze().park());
        assertTrue(r.analyze().leave());
        assertEquals(8, r.claimablePoints());
    }

    @Test
    public void sourcesRunOutAfterFourPollen() {
        AutoRoute r = AutoRoute.builder("f").start(35, FRONT, -90)
                .shootUp(4)
                .intakeGarden(5.9, 1.4, 9, FRONT + 0.8, -90, 4)
                .shootUp(4)
                .intakeGarden(5.9, 1.4, 9, FRONT + 0.8, -90, 4)
                .build();
        RouteAnalysis a = r.analyze();
        assertEquals("own GARDEN", a.step(2).sourceName);
        assertEquals(4, a.step(2).sourceRemainingBefore);
        assertEquals(0, a.step(4).sourceRemainingBefore);
        assertFalse(a.step(2).sourceOffLimits);
    }

    @Test
    public void intakesRecogniseFlowersAndTheOtherAlliancesSources() {
        AutoRoute r = AutoRoute.builder("g").start(35, FRONT, -90)
                .intakeFlower(2.4, 48, 4)
                .intakeFlower(96, 2.4, 1)
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
        AutoRoute red = AutoRoute.builder("h").start(35, FRONT, -90)
                .shootUp(4).intakeGarden(5.9, 1.4, 9, FRONT + 0.8, -90, 4).shootUp(4)
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
        assertEquals(BioBuzzConstants.Field.SIZE_IN - ea.x, eb.x, 1e-6);
        assertEquals(BioBuzzConstants.Field.SIZE_IN - ea.y, eb.y, 1e-6);
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
