package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * A FLOWER pick-up is a WIGGLE: the robot drives straight in off center so that one flat wheel is beside the bottom POLLEN, turns on the spot toward the ball
 * and back (once per POLLEN), and backs straight out. These tests pin down the poses it is planned from, and each rule the validator enforces on it:
 * <ul>
 *   <li>I1 nothing goes into a wall, I2 the CHASSIS never gets within its clearance of a plate, at ANY pose of the turn,</li>
 *   <li>I3 only the ONE working wheel may be over the plate, and it never goes nearer the wall than the POLLEN's center line,</li>
 *   <li>I4 the approach is square to the wall, straight and slow, and the retreat is straight (they are derived, so they hold by construction),</li>
 *   <li>I5 the wheel can reach the ball: if not, an ERROR that says by how many inches the wheels fall short.</li>
 * </ul>
 * Every validated route is checked for the BLUE half turn as well, which must give identical findings. Most tests use {@link WiggleTestRobot}, a robot whose
 * flat wheels stick out 4 in, because with the 2.7 in of the drawing the wiggle does not reach (see {@link FlowerWiggleTest}, and the test here that pins the
 * finding the shipped numbers give).
 */
public class FlowerPressTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double EPS = 1e-6;
    private static final AllianceField RED = new AllianceField(new FieldModel(), Alliance.RED);
    private static final FlowerWiggle.Spec LONG = WiggleTestRobot.spec();
    private static final double APPROACH = BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN;
    private static final double RETREAT = BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN;

    // ---- helpers ---------------------------------------------------------------------------------------------------

    /**
     * The 4 pre-loads are fired first (the volley tips the HIVE on the 3rd POLLEN and keeps the 4th), so the robot holds 1 and a FLOWER INTAKE of 3 is legal.
     * It then waits 5 s at the shooting pose for what the tipped cell drops to land (G409: the robot may not drive through the landing band during the
     * 3.5 s after the tip), and only then goes for the FLOWER. The INTAKE is step index 3.
     */
    private static AutoRoute.Builder base() {
        return AutoRoute.builder("press").start(35, FRONT, -90).shootUp(4).waitSeconds(5);
    }

    private static final int INTAKE = 3;
    private static final double[] FLOWER_1 = {BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1]};
    private static final double[] FLOWER_2 = {BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][1]};

    /** The way to the rear-wall FLOWER 2 along the left corridor, clear of the HIVE; its INTAKE is step index 4. */
    private static AutoRoute.Builder toRearFlower() {
        return AutoRoute.builder("rear").start(35, FRONT, -90).shootUp(4)
                .driveConstant(30.0, 45.0, -90.0).via(30.0, 24.0)
                .driveReverseTangent(30.0, 100.0);
    }

    /** Findings of one step for RED and for BLUE, which must agree (this robot has wheels that reach). */
    private static List<Issue> stepIssues(AutoRoute red, int step) {
        List<Issue> r = onStep(WiggleTestRobot.check(red), step);
        List<Issue> b = onStep(WiggleTestRobot.check(red.forAlliance(Alliance.BLUE)), step);
        assertEquals("red and blue findings differ:\n" + r + "\n" + b, signature(r), signature(b));
        return r;
    }

    private static List<Issue> onStep(List<Issue> all, int step) {
        List<Issue> out = new ArrayList<Issue>();
        for (Issue i : all) {
            if (i.stepIndex == step) {
                out.add(i);
            }
        }
        return out;
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
        fail("no " + severity + " for " + rule + " in " + issues);
        return null;
    }

    // ---- the poses are derived ---------------------------------------------------------------------------------------

    @Test
    public void thePosesOfAPickUpAreDerivedForEveryFlowerAndEitherWheel() {
        for (int f = 0; f < RED.flowerPlates().size(); f++) {
            for (IntakeWheel w : IntakeWheel.physical()) {
                FlowerPress p = FlowerPress.plan(RED, LONG, f, w, w, 3, RETREAT);
                FlowerWiggle.Solution s = p.solution;
                String where = "FLOWER " + (f + 1) + " " + w;
                assertEquals(where, f, p.flower);
                assertEquals(where, w, p.wheel);
                // facing the wall squarely at the line-up, the start and the retreat
                double facing = RED.flowerFacingHeading(f);
                assertEquals(where + " line-up heading", 0.0, Angles.distance(p.lineUp.heading, facing), EPS);
                assertEquals(where + " start heading", 0.0, Angles.distance(p.start.heading, facing), EPS);
                assertEquals(where + " retreat heading", 0.0, Angles.distance(p.retreat.heading, facing), EPS);
                // the start is the derived standoff from the wall; the line-up is the approach length farther out, the retreat the retreat length
                assertEquals(where + " standoff", s.standoffIn, RED.flowerWallDistance(f, new Point2(p.start.x, p.start.y)), EPS);
                assertEquals(where + " line-up distance", s.standoffIn + APPROACH, RED.flowerWallDistance(f, new Point2(p.lineUp.x, p.lineUp.y)), EPS);
                assertEquals(where + " retreat distance", s.standoffIn + RETREAT, RED.flowerWallDistance(f, new Point2(p.retreat.x, p.retreat.y)), EPS);
                // all three stand in one line along the wall's normal (the approach and the retreat are straight)
                assertEquals(where, s.lateralIn, RED.flowerLateral(f, new Point2(p.lineUp.x, p.lineUp.y)), EPS);
                assertEquals(where, s.lateralIn, RED.flowerLateral(f, new Point2(p.start.x, p.start.y)), EPS);
                assertEquals(where, s.lateralIn, RED.flowerLateral(f, new Point2(p.retreat.x, p.retreat.y)), EPS);
                assertEquals(where + " approach length", APPROACH, p.approach.length(), EPS);
                assertEquals(where + " retreat length", RETREAT, p.retreatPath.length(), EPS);
                // the working wheel starts 3.8 in beside the ball and ends 1.8 in from it, the pose point does not move during the turn
                assertEquals(where + " start offset", BioBuzzConstants.Auto.FLOWER_WHEEL_START_OFFSET_IN, Math.abs(RED.flowerLateral(f, p.wheelAtStart())), EPS);
                assertEquals(where + " pressed", BioBuzzConstants.Auto.FLOWER_WHEEL_PRESS_OFFSET_IN, p.wheelPushed().distanceTo(p.ball(RED)), 1e-9);
                assertEquals(where + " pose point stays put", 0.0, p.start.distanceTo(p.pushed), 0.0);
                assertEquals(where + " the turn", s.turnRad, Angles.error(p.start.heading, p.pushed.heading), EPS);
                assertEquals(where + " the swing is a turn on the spot", 0.0, p.swing.length(), EPS);
                assertEquals(where + " and its size is the derived turn", Math.abs(s.turnRad), p.swing.totalTurnRadians(), EPS);
                assertEquals(where + " cycles", 3, p.cycles);
                assertTrue(where, p.reaches());
                assertEquals(where + " the lever arm", 10.357, p.leverArmIn(), 0.001);
            }
        }
    }

    @Test
    public void theTwoWheelsStandOnOppositeSidesOfTheFlowerAndTurnOppositeWays() {
        FlowerPress left = FlowerPress.plan(RED, LONG, 0, IntakeWheel.LEFT, IntakeWheel.LEFT, 3, RETREAT);
        FlowerPress right = FlowerPress.plan(RED, LONG, 0, IntakeWheel.RIGHT, IntakeWheel.RIGHT, 3, RETREAT);
        double along = RED.flowerCenter(0).y;
        // facing the left wall (heading 180) the robot's left is -y: the LEFT wheel works when the robot stands toward +y of the ball's line, the RIGHT wheel from -y
        assertEquals(along + 2.3, left.start.y, 1e-9);
        assertEquals(along - 2.3, right.start.y, 1e-9);
        assertEquals(left.start.x, right.start.x, EPS);
        assertEquals(-left.turnRad(), right.turnRad(), EPS);
        assertTrue(left.turnRad() < 0.0 && right.turnRad() > 0.0);
    }

    @Test
    public void aFlowerIntakeHasNoAuthoredPoseAndItsWheelIsAutoUnlessTheRouteSaysOtherwise() {
        AutoRoute r = base().intakeFlower(FLOWER_1[0], FLOWER_1[1], 3).intakeFlower(FLOWER_1[0], FLOWER_1[1], IntakeWheel.RIGHT, 1).build();
        Step.Intake auto = (Step.Intake) r.step(INTAKE);
        Step.Intake right = (Step.Intake) r.step(INTAKE + 1);
        assertEquals(IntakeWheel.AUTO, auto.wheel());
        assertEquals(IntakeWheel.RIGHT, right.wheel());
        assertNull("the poses are derived, never authored", auto.robotPose());
        assertEquals("no dwell: the time comes from the wiggle", 0.0, auto.dwellSec(), 0.0);
        assertTrue(auto.describe(), auto.describe().contains("the cheaper flat wheel"));
        assertTrue(right.describe(), right.describe().contains("RIGHT wheel"));
        // a missing wheel means AUTO
        assertEquals(IntakeWheel.AUTO, Step.Intake.flower(new Point2(2.634, 47.281), 2, 6.0, null, "").wheel());
        // the robot's own wheel stays the robot's own wheel when the field is turned for BLUE
        Step.Intake blue = (Step.Intake) r.forAlliance(Alliance.BLUE).step(INTAKE + 1);
        assertEquals(IntakeWheel.RIGHT, blue.wheel());
        assertNull(blue.robotPose());
        assertEquals(BioBuzzConstants.Field.SIZE_IN - FLOWER_1[0], blue.target().x, 1e-9);
    }

    @Test
    public void aGardenOrFloorIntakeHasNoWheelAndIsNotAPickUpAndNeedsItsPose() {
        AutoRoute garden = AutoRoute.builder("g").start(35, FRONT, -90).shootUp(4).waitSeconds(5).intakeGarden(5.9, 1.4, 9, FRONT + 0.8, -90, 4).build();
        assertNull(((Step.Intake) garden.step(INTAKE)).wheel());
        assertNull(WiggleTestRobot.analyze(garden).step(INTAKE).press);
        try {
            new Step.Intake(IntakeSource.GARDEN, new Point2(5.9, 1.4), null, 2.0, 4, "");
            fail("a GARDEN INTAKE needs its robot pose");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("robot pose"));
        }
    }

    // ---- a working pick-up is clean ------------------------------------------------------------------------------------

    @Test
    public void aWorkingPickUpIsCleanForEveryOwnFlowerEitherWheelAndAuto() {
        for (IntakeWheel w : new IntakeWheel[] {IntakeWheel.AUTO, IntakeWheel.LEFT, IntakeWheel.RIGHT}) {
            List<Issue> one = stepIssues(base().intakeFlower(FLOWER_1[0], FLOWER_1[1], w, 3).build(), INTAKE);
            assertTrue("FLOWER 1 with " + w + ": " + one, one.isEmpty());
            List<Issue> two = stepIssues(toRearFlower().intakeFlower(FLOWER_2[0], FLOWER_2[1], w, 3).build(), 4);
            assertTrue("FLOWER 2 with " + w + ": " + two, two.isEmpty());
        }
    }

    @Test
    public void theBluePickUpIsTheHalfTurnOfTheRedOneWithTheSameWheelAndTheSameTime() {
        AutoRoute red = base().intakeFlower(FLOWER_1[0], FLOWER_1[1], IntakeWheel.RIGHT, 3).build();
        RouteAnalysis r = WiggleTestRobot.analyze(red);
        RouteAnalysis b = WiggleTestRobot.analyze(red.forAlliance(Alliance.BLUE));
        assertEquals(IntakeWheel.RIGHT, b.step(INTAKE).press.wheel);
        assertEquals(r.step(INTAKE).durationSec, b.step(INTAKE).durationSec, 1e-9);
        // the blue pose is the red pose turned half a turn about the field center, and the turn is the same
        assertEquals(BioBuzzConstants.Field.SIZE_IN - r.step(INTAKE).press.start.x, b.step(INTAKE).press.start.x, 1e-9);
        assertEquals(BioBuzzConstants.Field.SIZE_IN - r.step(INTAKE).press.start.y, b.step(INTAKE).press.start.y, 1e-9);
        assertEquals(r.step(INTAKE).press.turnRad(), b.step(INTAKE).press.turnRad(), 1e-12);
        // it works the blue alliance's own FLOWER 3 on the right wall
        assertEquals(2, b.step(INTAKE).press.flower);
    }

    // ---- the shipped numbers: the wheel falls short ----------------------------------------------------------------------

    @Test
    public void withTheShippedNumbersTheWheelsFallShortAndTheErrorSaysByHowManyInches() {
        AutoRoute forced = base().intakeFlower(FLOWER_1[0], FLOWER_1[1], IntakeWheel.LEFT, 3).build();
        List<Issue> l = onStep(RouteValidator.check(forced), INTAKE);
        Issue reach = first(l, Rule.FLOWER_PRESS, Severity.ERROR);
        assertTrue(reach.message, reach.message.contains("cannot reach the POLLEN"));
        assertTrue(reach.message, reach.message.contains("0.8 in more than they do"));
        assertTrue(reach.message, reach.message.contains("3.5 in in front of the chassis"));
        assertFalse(RouteValidator.isRunnable(RouteValidator.check(forced)));
        // with AUTO neither wheel is legal and the one finding says so, once, with the same figure
        AutoRoute auto = base().intakeFlower(FLOWER_1[0], FLOWER_1[1], 3).build();
        List<Issue> a = onStep(RouteValidator.check(auto), INTAKE);
        Issue neither = first(a, Rule.FLOWER_PRESS, Severity.ERROR);
        assertTrue(neither.message, neither.message.contains("neither wheel is legal"));
        assertTrue(neither.message, neither.message.contains("0.8 in more than they do"));
        assertEquals(a.toString(), 1, count(a, Rule.FLOWER_PRESS, Severity.ERROR));
        // the same route for a robot whose wheels reach 4 in has no finding at all: the ERROR is the wheels, not the route
        assertTrue(onStep(WiggleTestRobot.check(auto), INTAKE).isEmpty());
        // and a validator told that the wheels reach exactly what is missing agrees
        double need = FlowerWiggle.solve(IntakeWheel.LEFT, FlowerWiggle.Spec.fromConstants()).missingReachIn;
        RouteValidator justEnough = new RouteValidator(new FieldModel(), new TimingModel(), BioBuzzConstants.Robot.INTAKE_REACH_IN + need + 0.002);
        assertTrue(onStep(justEnough.validate(auto), INTAKE).isEmpty());
        RouteValidator aHairShort = new RouteValidator(new FieldModel(), new TimingModel(), BioBuzzConstants.Robot.INTAKE_REACH_IN + need - 0.002);
        assertEquals(1, count(onStep(aHairShort.validate(auto), INTAKE), Rule.FLOWER_PRESS, Severity.ERROR));
    }

    @Test
    public void aShortRetreatIsOnlyAWarning() {
        AutoRoute r = base().intakeFlower(FLOWER_1[0], FLOWER_1[1], IntakeWheel.LEFT, 3).retreat(2.0).build();
        List<Issue> issues = stepIssues(r, INTAKE);
        assertEquals(issues.toString(), 1, issues.size());
        assertEquals(Severity.WARN, issues.get(0).severity);
        assertEquals(Rule.FLOWER_PRESS, issues.get(0).rule);
        assertTrue(issues.get(0).message.contains("retreat is only 2.0 in"));
        assertEquals(2.0, WiggleTestRobot.analyze(r).step(INTAKE).press.retreatLengthIn, EPS);
        assertTrue(stepIssues(base().intakeFlower(FLOWER_1[0], FLOWER_1[1], IntakeWheel.LEFT, 3).build(), INTAKE).isEmpty());
    }

    // ---- the rules are swept over EVERY pose, the turn included -----------------------------------------------------------

    /** A pick-up whose standoff and turn are given, NOT derived: for showing what the rules say about a robot that forgot something. */
    private static FlowerPress handMade(IntakeWheel wheel, double standoff, double lateral, double turnRad) {
        FlowerWiggle.Solution s = new FlowerWiggle.Solution(wheel, LONG, standoff, lateral, turnRad, true, 1.8, 0.0, standoff, 10.357);
        return FlowerPress.plan(RED, LONG, 0, s, wheel, 3, RETREAT);
    }

    @Test
    public void aStandoffThatForgetsTheSwingClipsTheChassisAndTheDerivedOneDoesNot() {
        FlowerWiggle.Spec justEnough = FlowerWiggle.Spec.fromConstants().withIntakeReach(3.5);
        FlowerWiggle.Solution derived = FlowerWiggle.solve(IntakeWheel.LEFT, justEnough);
        FlowerPress good = FlowerPress.plan(RED, justEnough, 0, IntakeWheel.LEFT, IntakeWheel.LEFT, 3, RETREAT);
        LegSweeps clean = LegSweeps.run(RED, good.swing, good, false);
        assertFalse("the derived pick-up breaks nothing during the swing: " + clean.firstProblem(), clean.broken());
        // the straight press-in's standoff (the chassis front exactly on its clearance line), with the turn that is right for it
        double straight = derived.noSwingStandoffIn;
        double turn = FlowerWiggle.turnFor(justEnough, IntakeWheel.LEFT, straight, derived.lateralIn).phi;
        FlowerWiggle.Solution naive = new FlowerWiggle.Solution(IntakeWheel.LEFT, justEnough, straight, derived.lateralIn, turn, true, 1.8, 0.0, straight, 10.357);
        FlowerPress clipped = FlowerPress.plan(RED, justEnough, 0, naive, IntakeWheel.LEFT, 3, RETREAT);
        // before the turn the chassis is fine, during it the corner swings into the clearance zone: only the SWING leg catches it
        assertFalse(LegSweeps.run(RED, clipped.approach, clipped, false).chassis.broken());
        LegSweeps sweep = LegSweeps.run(RED, clipped.swing, clipped, false);
        assertTrue("the swing leg sees the chassis come within its clearance of the plate", sweep.chassis.broken());
        assertTrue("by more than half an inch: " + sweep.chassis.worst, sweep.chassis.worst > 0.5);
        assertTrue(sweep.firstProblem(), sweep.firstProblem().contains("chassis"));
    }

    @Test
    public void aWheelNearerTheWallThanThePollensCenterLineBreaksTheStopLineRule() {
        // the working wheel straight over the ball with its tip 1.2 in from the wall: well past the center line (2.634 in)
        FlowerPress deep = handMade(IntakeWheel.LEFT, FRONT - BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0 + 3.0, BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN, 0.0);
        LegSweeps s = LegSweeps.run(RED, deep.swing, deep, false);
        assertTrue(s.pressing.broken());
        assertTrue(s.firstProblem(), s.firstProblem().contains("stop line") || s.firstProblem().contains("chassis") || s.firstProblem().contains("plate"));
        // the derived pick-up keeps the wheel short of the line at every pose
        FlowerPress ok = FlowerPress.plan(RED, LONG, 0, IntakeWheel.LEFT, IntakeWheel.LEFT, 3, RETREAT);
        assertFalse(LegSweeps.run(RED, ok.swing, ok, false).pressing.broken());
    }

    @Test
    public void theOtherWheelMayNotBeOverThePlate() {
        // the working wheel is LEFT but the pose point stands so that the RIGHT wheel is on the ball's line: 6.1 in from the pose point on the other side
        FlowerPress bad = handMade(IntakeWheel.LEFT, 14.0, -BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN, 0.0);
        LegSweeps s = LegSweeps.run(RED, bad.approach, bad, false);
        assertTrue("the wheel that does not work enters the plate", s.otherWheel.broken());
        assertTrue(s.firstProblem(), s.firstProblem().contains("other wheel"));
    }

    @Test
    public void nothingMayGoThroughTheWallEvenWhileTurning() {
        // a standoff of 9 in puts the wheel tips (9.77 in in front of the pose point) 0.77 in into the wall
        FlowerPress through = handMade(IntakeWheel.LEFT, 9.0, 2.3, Math.toRadians(-10.0));
        LegSweeps s = LegSweeps.run(RED, through.swing, through, false);
        assertTrue(s.bounds.broken());
        assertTrue(s.firstProblem(), s.firstProblem().contains("leave the field"));
    }

    @Test
    public void aMoveToTheLineUpPoseThatRunsIntoTheHiveIsAnErrorOnBothWheels() {
        // straight from the shooting pose below the HIVE to the rear wall: through the HIVE keep-out
        AutoRoute r = base().intakeFlower(FLOWER_2[0], FLOWER_2[1], 3).build();
        List<Issue> l = onStep(WiggleTestRobot.check(r), INTAKE);
        assertTrue(l.toString(), count(l, Rule.HIVE_KEEPOUT, Severity.ERROR) >= 1);
        Issue neither = first(l, Rule.FLOWER_PRESS, Severity.ERROR);
        assertTrue(neither.message, neither.message.contains("neither wheel is legal"));
        assertTrue(neither.message, neither.message.contains("HIVE"));
        // the same pick-up reached by the corridor is clean
        assertTrue(onStep(WiggleTestRobot.check(toRearFlower().intakeFlower(FLOWER_2[0], FLOWER_2[1], 3).build()), 4).isEmpty());
    }

    @Test
    public void aFlowerIntakeOnTheOtherAlliancesWallIsStillAG402Error() {
        List<Issue> l = onStep(WiggleTestRobot.check(base().intakeFlower(BioBuzzConstants.Field.FLOWER_CENTERS_IN[2][0],
                BioBuzzConstants.Field.FLOWER_CENTERS_IN[2][1], 1).build()), INTAKE);
        assertTrue(l.toString(), count(l, Rule.SOURCE, Severity.ERROR) >= 1);
        assertTrue(first(l, Rule.SOURCE, Severity.ERROR).message.contains("opposing side"));
    }

    // ---- timing: the step is the move in, the approach, the wiggle and the retreat -----------------------------------

    @Test
    public void theIntakeStepEndsAtTheRetreatPoseAndCostsTheMoveInTheApproachTheWiggleAndTheRetreat() {
        AutoRoute r = base().intakeFlower(FLOWER_1[0], FLOWER_1[1], 3).build();
        RouteAnalysis a = WiggleTestRobot.analyze(r);
        RouteAnalysis.StepInfo info = a.step(INTAKE);
        FlowerPress p = info.press;
        assertNotNull(p);
        assertEquals("the step ends backed out of the FLOWER", p.retreat.x, info.to.x, EPS);
        assertEquals(p.retreat.y, info.to.y, EPS);
        TimingModel t = WiggleTestRobot.timing();
        double slow = BioBuzzConstants.Auto.FLOWER_APPROACH_SPEED_FRACTION;
        double approach = t.legSeconds(p.approach, slow);
        double retreat = t.legSeconds(p.retreatPath, slow);
        double wiggle = t.wiggleSeconds(p);
        assertTrue("the slow legs take time", approach > 0.0 && retreat > 0.0 && wiggle > 0.0);
        assertEquals(approach + wiggle + retreat, t.flowerSeconds(p), EPS);
        assertEquals("the pick-up as the analysis states it", t.flowerSeconds(p), info.pickupSec, EPS);
        assertEquals("the move to the line-up pose is the rest of the step", t.moveSeconds(info.from, p.lineUp) + info.pickupSec, info.durationSec, EPS);
        assertTrue(info.pickupText(), info.pickupText().startsWith("FLOWER pick-up: 3 POLLEN in about "));
        assertTrue(info.pickupText(), info.pickupText().endsWith(" s with the " + p.wheel + " wheel"));
        // slower than driving the same distance at full speed
        assertTrue(approach > t.legSeconds(p.approach, 1.0));
    }

    @Test
    public void theRouteAfterAFlowerStartsFromTheRetreatPose() {
        AutoRoute r = base().intakeFlower(FLOWER_1[0], FLOWER_1[1], 3).driveConstant(30, 70, 180).build();
        RouteAnalysis a = WiggleTestRobot.analyze(r);
        assertEquals(a.step(INTAKE).press.retreat.x, a.step(INTAKE + 1).from.x, EPS);
        assertEquals(a.step(INTAKE).press.retreat.y, a.step(INTAKE + 1).from.y, EPS);
    }

    @Test
    public void onlyAFlowerIntakeNearAFlowerIsAPickUp() {
        // a FLOWER intake whose target is near no FLOWER has nothing to wiggle
        AutoRoute nothing = AutoRoute.builder("n").start(35, FRONT, -90).shootUp(4).waitSeconds(5).intakeFlower(60, 30, IntakeWheel.LEFT, 1).build();
        RouteAnalysis a = WiggleTestRobot.analyze(nothing);
        assertNull(a.step(INTAKE).press);
        assertNull(a.step(INTAKE).choice);
        assertEquals("the robot stays where it is", a.step(INTAKE).from.x, a.step(INTAKE).to.x, EPS);
        assertEquals(0.0, a.step(INTAKE).durationSec, EPS);
        assertNull(FlowerPress.of((Step.Intake) nothing.step(INTAKE), RED));
        List<Issue> l = WiggleTestRobot.check(nothing);
        assertEquals(1, count(onStep(l, INTAKE), Rule.SOURCE, Severity.WARN));
    }
}
