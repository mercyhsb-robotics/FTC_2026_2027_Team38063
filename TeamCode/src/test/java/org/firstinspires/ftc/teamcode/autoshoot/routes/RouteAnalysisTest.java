package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

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
