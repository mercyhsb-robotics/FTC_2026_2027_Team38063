package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * Which flat wheel does a FLOWER pick-up: {@link WheelChoice#choose}, the one function that the validator, the diagram, the timing and (later) the robot's
 * runner all get their answer from. Either wheel can do the wiggle; AUTO (the default) picks by cost, LEFT or RIGHT forces it.
 *
 * <p>FLOWER 1 stands on the red wall at (2.634, 47.281). With the wheels reaching 4 in ({@link WiggleTestRobot}) the LEFT wheel's line-up pose is at y 49.58
 * (2.3 in toward +y of the ball) and the RIGHT wheel's at y 44.98 (2.3 in toward -y), both 19.8 in from the wall, so a robot coming from low y finds the RIGHT
 * wheel nearer and one coming from high y the LEFT wheel.
 */
public class WheelChoiceTest {
    private static final AllianceField RED = new AllianceField(new FieldModel(), Alliance.RED);
    private static final TimingModel TIMING = WiggleTestRobot.timing();
    private static final double[] FLOWER = {BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1]};
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double EPS = 1e-9;

    private static Step.Intake step(IntakeWheel wheel) {
        return (Step.Intake) AutoRoute.builder("x").start(35, FRONT, -90).intakeFlower(FLOWER[0], FLOWER[1], wheel, 3).build().step(1);
    }

    private static WheelChoice.Choice choose(IntakeWheel requested, Pose2 from, WheelChoice.Lookahead next) {
        return WheelChoice.choose(RED, TIMING, step(requested), 0, from, next);
    }

    private static Step.Drive drive(double x, double y, double headingDeg) {
        return (Step.Drive) AutoRoute.builder("d").start(20, 47, 180).driveConstant(x, y, headingDeg).build().step(1);
    }

    // ---- the cheaper wheel wins, both ways -------------------------------------------------------------------------------

    @Test
    public void aRobotComingFromLowYTakesTheRightWheelAndFromHighYTheLeftOne() {
        WheelChoice.Choice south = choose(IntakeWheel.AUTO, new Pose2(30, 30, Math.PI), WheelChoice.Lookahead.none());
        assertEquals(IntakeWheel.RIGHT, south.wheel());
        assertTrue(south.chosen().legal);
        assertTrue("the RIGHT wheel is the cheaper one by some hundredths of a second: " + south.savedSec(), south.savedSec() > 0.02);
        assertTrue(south.reason(), south.reason().startsWith("chosen automatically: ") && south.reason().contains("s shorter than the LEFT wheel"));
        WheelChoice.Choice north = choose(IntakeWheel.AUTO, new Pose2(30, 65, Math.PI), WheelChoice.Lookahead.none());
        assertEquals(IntakeWheel.LEFT, north.wheel());
        assertTrue(north.savedSec() > 0.02);
        assertTrue(north.reason(), north.reason().contains("s shorter than the RIGHT wheel"));
    }

    @Test
    public void theCostIsTheMoveInPlusThePickUpPlusTheMoveOutAndTheChoiceTakesTheSmallest() {
        Pose2 from = new Pose2(30, 30, Math.PI);
        WheelChoice.Lookahead next = WheelChoice.Lookahead.drive(drive(20, 80, 90));
        WheelChoice.Choice c = choose(IntakeWheel.AUTO, from, next);
        assertEquals(2, c.options().size());
        for (WheelChoice.Option o : c.options()) {
            FlowerPress p = o.press;
            assertEquals(o.wheel + " move in", TIMING.moveSeconds(from, p.lineUp), o.moveInSec, EPS);
            assertEquals(o.wheel + " pick-up", TIMING.flowerSeconds(p), o.pickupSec, EPS);
            assertEquals(o.wheel + " move out", TIMING.legSeconds(DrivePath.of(p.retreat, drive(20, 80, 90)), 1.0), o.moveOutSec, EPS);
            assertEquals(o.wheel + " cost", o.moveInSec + o.pickupSec + o.moveOutSec, o.costSec(), EPS);
            assertTrue(o.wheel + " legal", o.legal);
        }
        double smallest = Math.min(c.options().get(0).costSec(), c.options().get(1).costSec());
        assertEquals(smallest, c.chosen().costSec(), EPS);
        assertEquals(c.other().costSec() - c.chosen().costSec(), c.savedSec(), EPS);
    }

    @Test
    public void whatTheRobotDoesNextCanDecideTheWheelWhenWhereItComesFromCannot() {
        Pose2 onTheLine = new Pose2(30, FLOWER[1], Math.PI);        // exactly level with the FLOWER: both move-ins cost the same
        WheelChoice.Choice goingNorth = choose(IntakeWheel.AUTO, onTheLine, WheelChoice.Lookahead.drive(drive(20, 80, 90)));
        WheelChoice.Choice goingSouth = choose(IntakeWheel.AUTO, onTheLine, WheelChoice.Lookahead.drive(drive(20, 20, -90)));
        assertEquals("backing out toward +y is shorter from the LEFT wheel's stance", IntakeWheel.LEFT, goingNorth.wheel());
        assertEquals(IntakeWheel.RIGHT, goingSouth.wheel());
        assertTrue(goingNorth.savedSec() > 0.0 && goingSouth.savedSec() > 0.0);
        // a next step that is a pose (a SHOOT, a PARK) counts the same way
        WheelChoice.Choice toPose = choose(IntakeWheel.AUTO, onTheLine, WheelChoice.Lookahead.pose(new Pose2(25, 100, 0)));
        assertEquals(IntakeWheel.LEFT, toPose.wheel());
        // and so does another FLOWER pick-up (the nearer of its two wheels): FLOWER 2 is up the wall to +y
        Step.Intake next = (Step.Intake) AutoRoute.builder("n").start(35, FRONT, -90).intakeFlower(48.0, 138.714, 3).build().step(1);
        assertEquals(IntakeWheel.LEFT, choose(IntakeWheel.AUTO, onTheLine, WheelChoice.Lookahead.flower(next)).wheel());
    }

    @Test
    public void aTieGoesToTheRoomierWheelAndThenToTheLeftWheel() {
        // level with the FLOWER, nothing after: the two wheels are mirror images, same time, same room
        WheelChoice.Choice c = choose(IntakeWheel.AUTO, new Pose2(30, FLOWER[1], Math.PI), WheelChoice.Lookahead.none());
        assertEquals(c.options().get(0).costSec(), c.options().get(1).costSec(), 1e-9);
        assertEquals(c.options().get(0).clearanceIn, c.options().get(1).clearanceIn, 1e-9);
        assertEquals("the LEFT wheel wins an exact tie", IntakeWheel.LEFT, c.wheel());
        assertTrue(c.reason(), c.reason().contains("wins a tie"));
        assertEquals(0.0, c.savedSec(), 1e-9);
    }

    @Test
    public void asksWithNoPoseAreAnsweredToo() {
        // with no pose to come from (a quick look, or a tool) the move in costs nothing and the tie goes to LEFT
        WheelChoice.Choice c = choose(IntakeWheel.AUTO, null, WheelChoice.Lookahead.none());
        assertEquals(IntakeWheel.LEFT, c.wheel());
        assertEquals(0.0, c.chosen().moveInSec, 0.0);
    }

    // ---- forcing a wheel --------------------------------------------------------------------------------------------------

    @Test
    public void aWheelTheRouteForcesIsUsedEvenWhenTheOtherIsCheaper() {
        Pose2 south = new Pose2(30, 30, Math.PI);
        WheelChoice.Choice forced = choose(IntakeWheel.LEFT, south, WheelChoice.Lookahead.none());
        assertEquals(IntakeWheel.LEFT, forced.wheel());
        assertEquals("only the forced wheel is considered", 1, forced.options().size());
        assertTrue(forced.reason(), forced.reason().contains("asked for the LEFT wheel"));
        assertTrue(Double.isNaN(forced.savedSec()));
        WheelChoice.Choice auto = choose(IntakeWheel.AUTO, south, WheelChoice.Lookahead.none());
        assertTrue("forcing the dearer wheel costs time", forced.chosen().costSec() > auto.chosen().costSec());
    }

    // ---- legality comes first -----------------------------------------------------------------------------------------------

    @Test
    public void aWheelWhoseMoveInRunsIntoTheHiveIsNotLegalSoTheOtherOneIsChosenEvenIfDearer() {
        // from here the straight move to the LEFT wheel's line-up pose (y 49.6) clips the HIVE keep-out and the one to the RIGHT wheel's (y 45.0) does not
        Pose2 from = new Pose2(42, 36, Math.PI);
        WheelChoice.Choice c = choose(IntakeWheel.AUTO, from, WheelChoice.Lookahead.none());
        WheelChoice.Option left = c.options().get(0);
        WheelChoice.Option right = c.options().get(1);
        assertEquals(IntakeWheel.LEFT, left.wheel);
        assertFalse("the LEFT move in runs into the HIVE", left.legal);
        assertTrue(left.problem, left.problem.contains("HIVE"));
        assertTrue(right.legal);
        assertEquals(IntakeWheel.RIGHT, c.wheel());
        assertTrue(c.reason(), c.reason().contains("only legal one"));
        assertNull("no other legal wheel to compare with", c.otherLegal());
        assertTrue(c.anyLegal());
        // legality comes before cost: the answer does not depend on what the illegal wheel would have cost
        assertTrue("the chosen wheel is the legal one", c.chosen().legal);
    }

    @Test
    public void whenNeitherWheelIsLegalTheCheapestIsReturnedMarkedNotLegalWithTheReason() {
        // the shipped robot: wheels of 2.7 in cannot reach the ball, so neither wheel is legal
        Step.Intake auto = step(IntakeWheel.AUTO);
        WheelChoice.Choice c = WheelChoice.choose(RED, new TimingModel(), auto, 0, new Pose2(30, 30, Math.PI), WheelChoice.Lookahead.none());
        assertFalse(c.anyLegal());
        assertFalse(c.chosen().legal);
        assertNotNull(c.press());
        assertFalse(c.press().reaches());
        assertTrue(c.reason(), c.reason().startsWith("neither wheel is legal: "));
        assertTrue(c.reason(), c.reason().contains("0.8 in more than they do"));
        assertTrue("the cheaper of the two is still returned, so that a diagram and a time can be shown", c.wheel() == IntakeWheel.RIGHT);
    }

    // ---- one answer for everybody --------------------------------------------------------------------------------------------

    @Test
    public void theAnalysisTheValidatorAndTheTimingAllUseTheChoiceTheFunctionMakes() {
        AutoRoute r = AutoRoute.builder("one").start(35, FRONT, -90).shootUp(4).intakeFlower(FLOWER[0], FLOWER[1], 3).build();
        RouteAnalysis a = WiggleTestRobot.analyze(r);
        RouteAnalysis.StepInfo info = a.step(2);
        // asking the function with the same inputs gives the same wheel, the same poses and the same time
        WheelChoice.Choice again = WheelChoice.choose(a.field(), TIMING, (Step.Intake) r.step(2), 0, info.from, WheelChoice.Lookahead.none());
        assertEquals(again.wheel(), info.press.wheel);
        assertEquals(again.press().start.x, info.press.start.x, EPS);
        assertEquals(again.press().start.y, info.press.start.y, EPS);
        assertEquals(again.chosen().pickupSec, info.pickupSec, EPS);
        assertEquals(TIMING.flowerSeconds(info.press), info.pickupSec, EPS);
        assertSame("the step's choice is the object whose press the analysis keeps", info.choice.press(), info.press);
        // the validator judges that same wheel: a clean pick-up has no finding, and the timing in the gallery text is the same number
        assertTrue(onlyStep(WiggleTestRobot.check(r), 2).isEmpty());
        assertTrue(info.pickupText(), info.pickupText().contains(String.format(java.util.Locale.ROOT, "%.1f", info.pickupSec)));
    }

    @Test
    public void theBlueRouteMakesTheMirroredChoice() {
        AutoRoute r = AutoRoute.builder("one").start(35, FRONT, -90).shootUp(4).intakeFlower(FLOWER[0], FLOWER[1], 3).build();
        RouteAnalysis red = WiggleTestRobot.analyze(r);
        RouteAnalysis blue = WiggleTestRobot.analyze(r.forAlliance(Alliance.BLUE));
        // the half turn of the field turns the robot with it: its left wheel stays its left wheel, and the same wheel is the cheaper one
        assertEquals(red.step(2).press.wheel, blue.step(2).press.wheel);
        assertEquals(red.step(2).choice.savedSec(), blue.step(2).choice.savedSec(), 1e-9);
    }

    private static List<Issue> onlyStep(List<Issue> all, int step) {
        List<Issue> out = new java.util.ArrayList<Issue>();
        for (Issue i : all) {
            if (i.stepIndex == step) {
                out.add(i);
            }
        }
        return out;
    }

    @Test
    public void aRouteWithAutoMustBeValidForTheWheelItChoseAndOnlyErrorsWhenNeitherIsLegal() {
        // a route from the corridor: AUTO finds a legal wheel, so no ERROR
        AutoRoute good = AutoRoute.builder("g").start(35, FRONT, -90).shootUp(4).driveConstant(30.0, 45.0, -90.0).via(30.0, 24.0)
                .driveReverseTangent(30.0, 100.0).intakeFlower(48.0, 138.714, 3).build();
        assertTrue(onlyStep(WiggleTestRobot.check(good), 4).isEmpty());
        // the straight way from the shooting pose runs through the HIVE for both wheels: an ERROR that says neither wheel is legal
        AutoRoute bad = AutoRoute.builder("b").start(35, FRONT, -90).shootUp(4).intakeFlower(48.0, 138.714, 3).build();
        List<Issue> l = onlyStep(WiggleTestRobot.check(bad), 2);
        boolean neither = false;
        for (Issue i : l) {
            neither |= i.rule == Rule.FLOWER_PRESS && i.severity == Severity.ERROR && i.message.contains("neither wheel is legal");
        }
        assertTrue(l.toString(), neither);
    }
}
