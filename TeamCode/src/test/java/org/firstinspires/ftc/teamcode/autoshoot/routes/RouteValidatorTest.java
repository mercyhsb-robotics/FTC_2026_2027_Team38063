package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * One test per validator rule: a tiny route that breaks that rule (and, where the field allows it,
 * only that rule), plus the matching legal route that must pass. Every route is validated for RED and for
 * its BLUE half turn and both must give the same findings, which also tests the alliance handling.
 */
public class RouteValidatorTest {

    // ---- helpers -------------------------------------------------------------------------------------

    private static List<Issue> validate(AutoRoute.Builder b) {
        return validate(b.build());
    }

    /** Validates the route for RED and for BLUE and checks both give the same rule/severity/step findings. */
    private static List<Issue> validate(AutoRoute red) {
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

    private static void assertClean(List<Issue> issues) {
        assertTrue("expected no issues but got " + issues, issues.isEmpty());
    }

    /** At least one issue, and every issue is of this rule and severity. */
    private static void assertOnly(List<Issue> issues, Rule rule, Severity severity) {
        assertTrue("expected a " + severity + " for " + rule + " but got " + issues, count(issues, rule, severity) >= 1);
        for (Issue i : issues) {
            assertTrue("unexpected extra finding: " + i + "\nall: " + issues, i.rule == rule && i.severity == severity);
        }
    }

    private static Issue first(List<Issue> issues, Rule rule) {
        for (Issue i : issues) {
            if (i.rule == rule) {
                return i;
            }
        }
        fail("no issue for " + rule + " in " + issues);
        return null;
    }

    /** The legal baseline: C1 start, shoot, left corridor, park in the LOADING ZONE. */
    private static AutoRoute.Builder base() {
        return RouteAnalysisTest.corridorPark();
    }

    // ---- baseline ------------------------------------------------------------------------------------

    @Test
    public void theLegalBaselineHasNoFindings() {
        assertClean(validate(base()));
    }

    // ---- 1. START (G304) -------------------------------------------------------------------------------

    @Test
    public void theTeamStartsAreLegal() {
        assertClean(validate(AutoRoute.builder("b1").start(35.0, 9.2, -90).waitSeconds(1)));
        assertClean(validate(AutoRoute.builder("c1").start(58.0, 9.2, -90).waitSeconds(1)));
        assertClean(validate(AutoRoute.builder("a3").start(9.2112, 70.2702, 180).waitSeconds(1)));
        assertClean(validate(AutoRoute.builder("c6").start(61.5, 135.1, 90).waitSeconds(1)));
    }

    @Test
    public void aStartReachingIntoTheOpponentHalfIsAnError() {
        List<Issue> l = validate(AutoRoute.builder("x").start(70, 9.2, -90).waitSeconds(1));
        assertEquals(1, count(l, Rule.START_G304, Severity.ERROR));
        assertEquals(0, count(l, Rule.SIDE_G402, Severity.ERROR));
        assertTrue(first(l, Rule.START_G304).message.contains("opponent half"));
        assertEquals(0, first(l, Rule.START_G304).stepIndex);
    }

    @Test
    public void aStartAwayFromEveryWallIsAnError() {
        List<Issue> l = validate(AutoRoute.builder("x").start(35, 20, -90).waitSeconds(1));
        assertOnly(l, Rule.START_G304, Severity.ERROR);
        assertTrue(first(l, Rule.START_G304).message.contains("touch a perimeter wall"));
    }

    @Test
    public void wallContactToleranceIsAboutThreeQuartersOfAnInch() {
        // 0.3 in gap (the team poses) passes, 0.7 passes, 1.0 fails
        assertClean(validate(AutoRoute.builder("x").start(35, 8.9 + 0.7, -90).waitSeconds(1)));
        assertOnly(validate(AutoRoute.builder("x").start(35, 8.9 + 1.0, -90).waitSeconds(1)),
                Rule.START_G304, Severity.ERROR);
    }

    @Test
    public void aStartInTheOwnLoadingZoneIsAnError() {
        List<Issue> l = validate(AutoRoute.builder("x").start(8.9, 108, 180).waitSeconds(1));
        assertOnly(l, Rule.START_G304, Severity.ERROR);
        assertTrue(first(l, Rule.START_G304).message.contains("LOADING ZONE"));
    }

    @Test
    public void aStartTouchingAFlowerIsAnError() {
        List<Issue> l = validate(AutoRoute.builder("x").start(8.9, 48, 180).waitSeconds(1));
        assertOnly(l, Rule.START_G304, Severity.ERROR);
        assertTrue(first(l, Rule.START_G304).message.contains("FLOWER 1"));
    }

    @Test
    public void aStartOutsideTheFieldOrAttachedToTheHiveIsAnError() {
        List<Issue> outside = validate(AutoRoute.builder("x").start(-5, 70, 180).waitSeconds(1));
        assertTrue(first(outside, Rule.START_G304).message.contains("out of the field"));
        // the HIVE frame is in the middle of the field so no wall-touching start can reach it; the check still fires
        List<Issue> hive = validate(AutoRoute.builder("x").start(60, 70, 0).waitSeconds(1));
        boolean found = false;
        for (Issue i : hive) {
            found |= i.rule == Rule.START_G304 && i.message.contains("HIVE frame");
        }
        assertTrue(hive.toString(), found);
    }

    // ---- 2. legs: field bounds, HIVE, FLOWERS ------------------------------------------------------------

    @Test
    public void aLegThatLeavesTheFieldIsAnError() {
        List<Issue> l = validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, -3, -90));
        assertOnly(l, Rule.FIELD_BOUNDS, Severity.ERROR);
        assertEquals(1, first(l, Rule.FIELD_BOUNDS).stepIndex);
    }

    @Test
    public void onlyAStepMarkedWallContactMayGrazeTheWall() {
        // center y = 8.0 puts the front edge 0.9 in through the wall: more than the 0.1 in tolerance, less than the 1.0 in allowance
        assertOnly(validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, 8.0, -90)),
                Rule.FIELD_BOUNDS, Severity.ERROR);
        assertClean(validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, 8.0, -90).wallContact()));
        // but not a full 2 in through
        assertOnly(validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, 6.9, -90).wallContact()),
                Rule.FIELD_BOUNDS, Severity.ERROR);
    }

    @Test
    public void turningInPlaceAgainstTheWallCountsToo() {
        // the old B1 hold: -90 -> -115 degrees at y = 9.2 swings a corner through the audience wall; the square
        // 17.80 in robot reaches 8.9 * (sin 115 + cos 115 in magnitude) = 11.83 in below its center, 2.63 in too far
        List<Issue> l = validate(AutoRoute.builder("x").start(35, 9.2, -90).shootUp(35, 9.2, -115, 4));
        assertEquals(1, count(l, Rule.FIELD_BOUNDS, Severity.ERROR));
        assertEquals(1, first(l, Rule.FIELD_BOUNDS).stepIndex);
        double halfLength = BioBuzzConstants.Robot.LENGTH_IN / 2.0;
        double halfWidth = BioBuzzConstants.Robot.WIDTH_IN / 2.0;
        double overshoot = halfLength * Math.abs(Math.sin(Math.toRadians(115))) + halfWidth * Math.abs(Math.cos(Math.toRadians(115))) - 9.2;
        assertEquals(2.63, overshoot, 0.005);
        assertTrue(first(l, Rule.FIELD_BOUNDS).message, first(l, Rule.FIELD_BOUNDS).message
                .contains(String.format(java.util.Locale.ROOT, "%.1f in", overshoot)));
    }

    @Test
    public void aLegThroughTheHiveFrameIsAnError() {
        List<Issue> l = validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(58, 118, -90));
        assertOnly(l, Rule.HIVE_KEEPOUT, Severity.ERROR);
        assertTrue(first(l, Rule.HIVE_KEEPOUT).message.contains("keep-out"));
    }

    @Test
    public void theHiveKeepOutIncludesTheMargin() {
        // Driving up the left corridor with the robot facing -y: its x half-extent is half the 17.80 in width (8.9 in).
        // The frame starts at x = 72 - 49.46 / 2 = 47.27 and the keep-out 3.0 in before it, at 44.27, so the center may
        // be at most 44.27 - 8.9 = 35.37 in from the left wall.
        double frameMinX = BioBuzzConstants.Field.SIZE_IN / 2.0 - BioBuzzConstants.Field.HIVE_FRAME_WIDTH_X_IN / 2.0;
        double limit = frameMinX - BioBuzzConstants.Auto.HIVE_KEEPOUT_MARGIN_IN - BioBuzzConstants.Robot.WIDTH_IN / 2.0;
        assertEquals(35.37, limit, 0.005);
        assertClean(validate(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(35.3, 90, -90)));
        // 0.13 in past the limit is inside the margin: the old B1 path1 target x = 35.8 was chosen for a narrower robot
        assertOnly(validate(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(35.5, 90, -90)),
                Rule.HIVE_KEEPOUT, Severity.ERROR);
        assertOnly(validate(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(37.0, 90, -90)),
                Rule.HIVE_KEEPOUT, Severity.ERROR);
    }

    @Test
    public void aLegAlongTheWallThroughAFlowerPlateIsAnError() {
        // start A3, drive straight down the left wall past FLOWER 1 (y 45..51)
        List<Issue> l = validate(AutoRoute.builder("x").start(8.9, 61.5, 180).driveConstant(9, 30, 180));
        assertOnly(l, Rule.FLOWER_KEEPOUT, Severity.ERROR);
        assertTrue(first(l, Rule.FLOWER_KEEPOUT).message.contains("FLOWER 1"));
    }

    @Test
    public void theFlowerAnIntakeTargetsIsExemptFromTheKeepOut() {
        // the same approach to (13.5, 48, 180): a plain drive hits the plate, an intake there is the whole point
        AutoRoute.Builder plain = AutoRoute.builder("x").start(8.9, 61.5, 180).driveConstant(13.5, 62, 180)
                .driveConstant(13.5, 48, 180);
        assertOnly(validate(plain), Rule.FLOWER_KEEPOUT, Severity.ERROR);

        AutoRoute.Builder intake = AutoRoute.builder("x").start(8.9, 61.5, 180).driveConstant(13.5, 62, 180)
                .driveConstant(13.5, 48, 180)
                .intakeFlower(2.4, 48, 13.5, 48, 180, 4);
        // the drive that ends at the FLOWER is exempt too; one FLOWER-intake with the 4 pre-loads held is over capacity
        List<Issue> l = validate(intake);
        assertEquals(0, count(l, Rule.FLOWER_KEEPOUT, Severity.ERROR));
    }

    // ---- 3. G402 sides -----------------------------------------------------------------------------------

    @Test
    public void theRobotCenterMustStayOnTheOwnSide() {
        List<Issue> l = validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(75, 9.2, -90));
        assertOnly(l, Rule.SIDE_G402, Severity.ERROR);
        assertEquals(1, first(l, Rule.SIDE_G402).stepIndex);
        // the center exactly on the line is allowed by G402; the body overhanging it is only the end-pose warning
        List<Issue> onLine = validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(72, 9.2, -90));
        assertEquals(0, count(onLine, Rule.SIDE_G402, Severity.ERROR));
        assertOnly(onLine, Rule.END_POSE, Severity.WARN);
    }

    // ---- 4. POLLEN (G407 and sources) --------------------------------------------------------------------

    @Test
    public void holdingMoreThanFourPollenIsAnError() {
        // 4 pre-loads + 4 from the GARDEN = 8
        List<Issue> l = validate(AutoRoute.builder("x").start(35, 9.2, -90)
                .driveConstant(9, 9.7, -90).wallContact()
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4));
        assertOnly(l, Rule.CAPACITY_G407, Severity.ERROR);
        assertTrue(first(l, Rule.CAPACITY_G407).message.contains("8 POLLEN"));
    }

    @Test
    public void firingPollenTheRobotDoesNotHoldIsAnError() {
        // (the 4 pre-loads tip the HIVE, so the second volley goes to the other cell: that move has findings of its own)
        List<Issue> none = validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4).shootUp(1));
        assertEquals(1, count(none, Rule.CAPACITY_G407, Severity.ERROR));
        assertEquals(2, first(none, Rule.CAPACITY_G407).stepIndex);
        assertTrue(first(none, Rule.CAPACITY_G407).message.contains("holds none"));

        // (5 POLLEN into a cell that tips on the 4th also fires one past the tip: that is its own G417 finding)
        List<Issue> tooMany = validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(5));
        assertEquals(1, count(tooMany, Rule.CAPACITY_G407, Severity.ERROR));
        assertTrue(first(tooMany, Rule.CAPACITY_G407).message.contains("only 4"));
    }

    @Test
    public void aFullCycleStaysWithinCapacity() {
        // shoot 4, take 4 from the GARDEN, shoot 4: never more than 4 held
        AutoRoute.Builder b = AutoRoute.builder("x").start(58, 9.2, -90)
                .shootUp(4)
                .driveConstant(9, 18, -90)
                .driveConstant(9, 9.7, -90).wallContact()
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4)
                .shootUp(4);
        assertEquals(0, count(validate(b), Rule.CAPACITY_G407, Severity.ERROR));
    }

    @Test
    public void takingFromAnEmptySourceIsAWarning() {
        AutoRoute.Builder b = AutoRoute.builder("x").start(58, 9.2, -90)
                .shootUp(4)
                .driveConstant(9, 9.7, -90).wallContact()
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4)
                .shootUp(4)
                .driveConstant(9, 9.7, -90).wallContact()
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4);
        List<Issue> l = validate(b);
        assertEquals(1, count(l, Rule.SOURCE, Severity.WARN));
        assertTrue(first(l, Rule.SOURCE).message.contains("only 0 POLLEN are left"));
        assertEquals(0, count(l, Rule.SOURCE, Severity.ERROR));
    }

    @Test
    public void takingFromTheOtherAlliancesSourcesIsAnError() {
        // robot pose on the own side so that only the source is wrong: FLOWER 4 sits on the audience wall at x = 96 (blue's)
        List<Issue> flower = validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4)
                .driveConstant(70, 12, 0).intakeFlower(96, 2.4, 70, 12, 0, 1));
        assertEquals(1, count(flower, Rule.SOURCE, Severity.ERROR));
        assertTrue(first(flower, Rule.SOURCE).message.contains("opposing side"));
        List<Issue> garden = validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4)
                .driveConstant(70, 12, 0).intakeGarden(135, 138, 70, 12, 0, 1));
        assertEquals(1, count(garden, Rule.SOURCE, Severity.ERROR));
    }

    @Test
    public void anIntakeWhoseTargetIsNearNoSourceIsAWarning() {
        List<Issue> l = validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4)
                .driveConstant(30, 30, 0).intakeFlower(60, 30, 30, 30, 0, 1));
        assertEquals(1, count(l, Rule.SOURCE, Severity.WARN));
        assertTrue(first(l, Rule.SOURCE).message.contains("nothing to take"));
    }

    @Test
    public void aFloorIntakeNeedsSomethingThatCouldHaveLandedThere() {
        AutoRoute.Builder before = AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(30, 30, 0).intakeFloor(34, 30, 30, 30, 0, 1);
        assertEquals("nothing was shot yet", 1, count(validate(before), Rule.SOURCE, Severity.WARN));
        AutoRoute.Builder after = AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4)
                .driveConstant(30, 30, 0).intakeFloor(34, 30, 30, 30, 0, 1);
        assertEquals(0, count(validate(after), Rule.SOURCE, Severity.WARN));
    }

    // ---- 5. time (G401) ----------------------------------------------------------------------------------

    @Test
    public void theTimeLimitIsAnErrorBeyond30SecondsAndAWarningInsideTheMargin() {
        assertClean(validate(AutoRoute.builder("x").start(58, 9.2, -90).waitSeconds(20)));
        List<Issue> tight = validate(AutoRoute.builder("x").start(58, 9.2, -90).waitSeconds(29.0));
        assertOnly(tight, Rule.TIME_G401, Severity.WARN);
        assertEquals(-1, first(tight, Rule.TIME_G401).stepIndex);
        assertOnly(validate(AutoRoute.builder("x").start(58, 9.2, -90).waitSeconds(31.0)), Rule.TIME_G401, Severity.ERROR);
        // exactly 30 s is still legal but inside the margin
        assertOnly(validate(AutoRoute.builder("x").start(58, 9.2, -90).waitSeconds(30.0)), Rule.TIME_G401, Severity.WARN);
        assertClean(validate(AutoRoute.builder("x").start(58, 9.2, -90).waitSeconds(27.9)));
    }

    // ---- 6. end pose -------------------------------------------------------------------------------------

    @Test
    public void endingInsideTheHiveFrameIsAnError() {
        List<Issue> l = validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(60, 70, 0));
        assertEquals(1, count(l, Rule.END_POSE, Severity.ERROR));
        assertEquals("the last step", 1, first(l, Rule.END_POSE).stepIndex);
        // getting there also crossed the keep-out
        assertTrue(count(l, Rule.HIVE_KEEPOUT, Severity.ERROR) >= 1);
    }

    @Test
    public void endingWithTheBodyOverTheCenterLineIsAWarning() {
        AutoRoute.Builder b = AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4)
                .driveReverseTangent(30, 30).driveReverseTangent(30, 110).driveConstant(66, 110, 0);
        List<Issue> l = validate(b);
        assertOnly(l, Rule.END_POSE, Severity.WARN);
    }

    // ---- 7. SHOOT pose and target ------------------------------------------------------------------------

    @Test
    public void aShootFromThePoseTheSolverWouldChooseIsFine() {
        assertClean(validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4)));
        assertClean(validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(58, 9.2, -90, 4)));
    }

    @Test
    public void aShootOutsideTheStandoffBandIsAWarning() {
        // the old B1 hold pose is 43 in from the cell; it also swings through the wall (a different rule)
        List<Issue> far = validate(AutoRoute.builder("x").start(35, 9.2, -90).shootUp(35, 9.2, -115, 4));
        assertEquals(1, count(far, Rule.SHOOT_POSE, Severity.WARN));
        assertTrue(first(far, Rule.SHOOT_POSE).message.contains("standoff band"));
        assertTrue(first(far, Rule.SHOOT_POSE).message.contains("43.4"));
        // far too close to the HIVE
        List<Issue> near = validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(59.25, 30, -90)
                .shootUp(59.25, 30, -90, 4));
        assertEquals(1, count(near, Rule.SHOOT_POSE, Severity.WARN));
        assertEquals(0, count(near, Rule.SHOOT_POSE, Severity.ERROR));
    }

    @Test
    public void aShootWithTheBackTurnedAwayFromTheCellIsAWarning() {
        // C1 start has the back to the cell; turning around the front toward it
        List<Issue> l = validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(59.25, 14, 90)
                .shootUp(59.25, 14, 90, 4));
        boolean aim = false;
        for (Issue i : l) {
            aim |= i.rule == Rule.SHOOT_POSE && i.message.contains("deg away from the aim point");
        }
        assertTrue(l.toString(), aim);
    }

    @Test
    public void aShootFromTheWrongSideOfTheHiveIsAWarning() {
        // a robot at the rear wall (legal C6 start) cannot shoot the audience-side UP cell
        List<Issue> l = validate(AutoRoute.builder("x").start(61.5, 135.1, 90).shootUp(61.5, 135.1, 90, 4));
        boolean side = false;
        for (Issue i : l) {
            side |= i.rule == Rule.SHOOT_POSE && i.message.contains("wrong side");
        }
        assertTrue(l.toString(), side);
    }

    @Test
    public void shootingAtACellThatIsDownIsAWarningAndTheHiveStateFlipsAfterATip() {
        // at match start the red AUDIENCE cell is UP: a fixed REAR shot is at the DOWN cell
        List<Issue> down = validate(AutoRoute.builder("x").start(58, 9.2, -90).shootSide(Side.REAR, 4));
        assertEquals(1, count(down, Rule.SHOOT_TARGET, Severity.WARN));
        assertTrue(first(down, Rule.SHOOT_TARGET).message.contains("REAR"));
        // fixed AUDIENCE is right at the start...
        assertEquals(0, count(validate(AutoRoute.builder("x").start(58, 9.2, -90).shootSide(Side.AUDIENCE, 4)),
                Rule.SHOOT_TARGET, Severity.WARN));
        // ...but once the 4 pre-loads have tipped the HIVE (3 staged NECTAR + 4 POLLEN) the REAR cell is the UP one and
        // AUDIENCE is down; the second volley of 4 into the empty REAR cell does not tip it again
        AutoRoute.Builder afterTip = AutoRoute.builder("x").start(58, 9.2, -90)
                .shootUp(4)
                .driveConstant(9, 9.7, -90).wallContact()
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4)
                .shootUp(4)
                .driveConstant(9, 9.7, -90).wallContact()
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4)
                .shootSide(Side.AUDIENCE, 4);
        assertEquals(1, count(validate(afterTip), Rule.SHOOT_TARGET, Severity.WARN));
    }

    // ---- 8. PARK -----------------------------------------------------------------------------------------

    @Test
    public void aParkOutsideTheLoadingZoneIsAnError() {
        List<Issue> l = validate(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, 30, -90).park(30, 30, -90));
        assertOnly(l, Rule.PARK_ZONE, Severity.ERROR);
        assertEquals(2, first(l, Rule.PARK_ZONE).stepIndex);
    }

    @Test
    public void aShallowParkIsAWarningADeepOneIsFine() {
        // the old B1 park: the 17.80 in square robot at (12, 90, 0) reaches y 98.9, 2.9 in into the zone (y from 96)
        AutoRoute.Builder shallow = AutoRoute.builder("x").start(35, 9.2, -90)
                .driveReverseTangent(34, 50, -90).driveReverseTangent(30, 90, -90).driveReverseTangent(12, 90, 0).park(12, 90, 0);
        List<Issue> l = validate(shallow);
        assertOnly(l, Rule.PARK_ZONE, Severity.WARN);
        assertTrue(first(l, Rule.PARK_ZONE).message, first(l, Rule.PARK_ZONE).message.contains("only 2.9 in"));
        // 12 in deeper in y: 7.9 in overlap
        assertClean(validate(AutoRoute.builder("x").start(35, 9.2, -90)
                .driveReverseTangent(34, 50, -90).driveReverseTangent(30, 90, -90).driveLinear(12, 108, 0).park(12, 108, 0)));
    }

    // ---- 9. unplanned moves ------------------------------------------------------------------------------

    @Test
    public void anExplicitPoseFarFromWhereTheRobotIsWantsADriveFirst() {
        List<Issue> l = validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4)
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4));
        // (the shoot pose is AUTO: the solver drives, no warning; the intake pose is explicit and 50 in away)
        assertEquals(1, count(l, Rule.IMPLICIT_MOVE, Severity.WARN));
        assertEquals(2, first(l, Rule.IMPLICIT_MOVE).stepIndex);
        // with a DRIVE first the warning is gone
        assertEquals(0, count(validate(AutoRoute.builder("x").start(58, 9.2, -90).shootUp(4)
                .driveConstant(9, 18, -90).driveConstant(9, 9.7, -90).wallContact()
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4)), Rule.IMPLICIT_MOVE, Severity.WARN));
    }

    // ---- API ---------------------------------------------------------------------------------------------

    @Test
    public void issuesAreSortedByStepAndKnowTheirManualRule() {
        List<Issue> l = validate(AutoRoute.builder("x").start(35, 9.2, -90).shootUp(35, 9.2, -115, 4).waitSeconds(40));
        int previous = -2;
        for (Issue i : l) {
            int idx = i.stepIndex < 0 ? Integer.MAX_VALUE : i.stepIndex;
            assertTrue("sorted by step: " + l, idx >= previous);
            previous = idx;
            assertFalse(i.rule.manualRule().isEmpty());
            assertFalse(i.message.isEmpty());
        }
        assertEquals("the whole-route time issue comes last", -1, l.get(l.size() - 1).stepIndex);
        assertFalse(RouteValidator.isRunnable(l));
        assertTrue(RouteValidator.isRunnable(validate(base())));
        Issue bounds = first(l, Rule.FIELD_BOUNDS);
        assertEquals(2, bounds.stepNumber());
        assertTrue(bounds.toString(), bounds.toString().startsWith("ERROR step 2 [9.2/G412]"));
    }
}
