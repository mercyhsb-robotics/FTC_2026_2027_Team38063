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
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * Taking the bottom POLLEN out of a FLOWER is a PRESS-IN: the robot faces the wall, drives straight in until ONE flat intake
 * wheel overlaps the POLLEN, runs the intake while pressed, then backs straight out. These tests pin down the geometry
 * (where the wheel, the chassis and the plate are) and each invariant the validator enforces:
 * <ul>
 *   <li>I1 nothing goes into a wall, I2 the chassis never touches a plate,</li>
 *   <li>I3 only the ONE pressing wheel disc may enter the plate, and only up to the press stop line,</li>
 *   <li>I4 square to the wall, wheel on the center line, slow straight approach, dwell, straight retreat.</li>
 * </ul>
 * Every validated route is also checked for the BLUE half turn, which must give identical findings.
 *
 * <p>Numbers (defaults, official CAD): plate 4.871 x 5.95 in with its field-side edge 5.033 in from the wall; POLLEN 2.8 in
 * across, its center 2.634 in from the wall, so it spans 1.234 to 4.034 in; the wheels reach 2.7 in in front of the chassis
 * and are 12.2 in apart; the chassis stands 0.5 in off the plate (5.533 in from the wall), so the wheel tip stops
 * 5.533 - 2.7 = 2.833 in from the wall, 1.201 in into the POLLEN.
 */
public class FlowerPressTest {
    /** The robot's center of rotation is this far (9.77 in) from a wall its FRONT edge is touching; see RobotShape. */
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double EPS = 1e-6;
    private static final double REACH = BioBuzzConstants.Robot.INTAKE_REACH_IN;
    /** How far the plate's field-side edge is from the wall (the plate itself is 4.871 in deep and stands 0.162 in off the wall). */
    private static final double PLATE_DEPTH = BioBuzzConstants.Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN;
    private static final double CLEARANCE = BioBuzzConstants.Auto.FLOWER_CHASSIS_CLEARANCE_IN;
    private static final AllianceField RED = new AllianceField(new FieldModel(), Alliance.RED);

    // ---- helpers -------------------------------------------------------------------------------------

    private static AutoRoute.Builder base() {
        // The 4 pre-loads are fired first: the volley tips the HIVE on the 3rd POLLEN and keeps the 4th (the spare), so the robot
        // holds 1 and a FLOWER INTAKE of 3 is legal; the drive to the FLOWER is not what these tests judge
        return AutoRoute.builder("press").start(35, FRONT, -90).shootUp(4);
    }

    /** A route that drives to the given line-up pose and presses with the given wheel; the INTAKE is step index 3. */
    private static AutoRoute lineUpRoute(int flower, IntakeWheel wheel, Pose2 lineUp) {
        Point2 c = RED.flowerCenter(flower);
        return base().driveConstant(lineUp.x, lineUp.y, lineUp.headingDegrees())
                .intakeFlower(c.x, c.y, lineUp.x, lineUp.y, lineUp.headingDegrees(), wheel, 3).build();
    }

    private static final int INTAKE = 3;

    /** Findings of the FLOWER INTAKE step (index 3) for RED and for BLUE, which must agree. */
    private static List<Issue> intakeIssues(AutoRoute red) {
        List<Issue> r = onStep(RouteValidator.check(red), INTAKE);
        List<Issue> b = onStep(RouteValidator.check(red.forAlliance(Alliance.BLUE)), INTAKE);
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

    private static Pose2 ideal(int flower, IntakeWheel wheel) {
        return FlowerPress.idealLineUp(RED, flower, wheel);
    }

    private static FlowerPress press(AutoRoute r) {
        return FlowerPress.of((Step.Intake) r.step(INTAKE), r.analyze().field());
    }

    /** The two FLOWERS on the red half (index 0: the left wall, index 1: the rear wall). */
    private static final int[] OWN_FLOWERS = {0, 1};

    // ---- the numbers fit together --------------------------------------------------------------------------

    @Test
    public void theDefaultNumbersAreConsistentWithEachOther() {
        double pollenEdge = BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN + BioBuzzConstants.Auto.POLLEN_DIAMETER_IN / 2.0;
        assertEquals("the POLLEN's field-side edge", 4.034, pollenEdge, EPS);
        assertEquals("the wheel tip stops 1.201 in inside the POLLEN's edge", 2.833, RED.flowerPressStopLineIn(), EPS);
        assertTrue("never closer than the POLLEN's center line",
                RED.flowerPressStopLineIn() >= BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN - EPS);
        assertEquals("the chassis front then stands 0.5 in off the plate (its edge is 5.033 in from the wall)", PLATE_DEPTH + CLEARANCE,
                RED.flowerPressStopLineIn() + REACH, EPS);
        assertTrue("so the real reach is just enough: the required reach is the default reach",
                RED.flowerRequiredReachIn() <= REACH + EPS);
        assertEquals(2.7, RED.flowerRequiredReachIn(), EPS);
        // the two wheels cannot straddle the POLLEN: they are far wider apart than it is wide
        assertTrue(2.0 * BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN > 4.0 * BioBuzzConstants.Auto.POLLEN_DIAMETER_IN);
    }

    @Test
    public void thePressZoneIsThePartOfThePlateNearerTheWallThanTheStopLine() {
        for (int f = 0; f < RED.flowerPlates().size(); f++) {
            Rect zone = RED.flowerPressZone(f);
            Rect plate = RED.flowerPlate(f);
            assertTrue(zone.minX >= plate.minX - EPS && zone.maxX <= plate.maxX + EPS && zone.minY >= plate.minY - EPS
                    && zone.maxY <= plate.maxY + EPS);
            // its depth is the stop line, its length the whole plate
            double depth = RED.flowerWallDistance(f, new Point2((zone.minX + zone.maxX) / 2.0, (zone.minY + zone.maxY) / 2.0));
            assertTrue(depth > 0.0 && depth < RED.flowerPressStopLineIn());
            assertEquals(BioBuzzConstants.Field.FLOWER_PLATE_ALONG_WALL_IN * RED.flowerPressStopLineIn(),
                    (zone.maxX - zone.minX) * (zone.maxY - zone.minY), EPS);
        }
    }

    // ---- the derived poses ---------------------------------------------------------------------------------

    @Test
    public void thePressPoseHasTheWheelOnThePollenAndTheChassisOnItsClearanceLine() {
        for (int f = 0; f < RED.flowerPlates().size(); f++) {
            for (IntakeWheel w : IntakeWheel.physical()) {
                Pose2 lineUp = ideal(f, w);
                Point2 c = RED.flowerCenter(f);
                Step.Intake step = new Step.Intake(IntakeSource.FLOWER, c, lineUp, 3.0, 4, BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN, w, "");
                FlowerPress p = FlowerPress.of(step, RED);
                assertNotNull(p);
                assertEquals(f, p.flower);
                assertEquals(w, p.wheel);
                // facing the wall squarely
                assertEquals(0.0, p.headingErrorDeg(), EPS);
                assertEquals(0.0, Math.toDegrees(Math.abs(Angles.error(p.press.heading, RED.flowerFacingHeading(f)))), EPS);
                // the middle of the pressing wheel is on the FLOWER's center line, its tip on the press stop line
                Point2 wheelAtPress = Footprint.wheelCenter(p.press, w);
                assertEquals("lateral", 0.0, RED.flowerLateral(f, wheelAtPress), EPS);
                assertEquals("lateral at the line-up", 0.0, p.lateralIn, EPS);
                double tip = RED.flowerWallDistance(f, wheelAtPress) - BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0;
                assertEquals("wheel tip from the wall", RED.flowerPressStopLineIn(), tip, EPS);
                // the chassis front is REACH behind the wheel tip: exactly plate + clearance from the wall
                double chassisFront = RED.flowerWallDistance(f, new Point2(p.press.x, p.press.y)) - (Footprint.frontExtentIn() - REACH);
                assertEquals("chassis front from the wall", PLATE_DEPTH + CLEARANCE, chassisFront, EPS);
                // the final approach is the approach length, the retreat is straight out by the retreat length
                assertEquals(BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN, p.approachAxialIn, EPS);
                assertEquals(BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN, p.press.distanceTo(p.retreat), EPS);
                assertEquals(p.retreatLengthIn, RED.flowerWallDistance(f, new Point2(p.retreat.x, p.retreat.y))
                        - RED.flowerWallDistance(f, new Point2(p.press.x, p.press.y)), EPS);
                assertEquals("retreat keeps the heading", p.press.heading, p.retreat.heading, EPS);
            }
        }
    }

    @Test
    public void theRobotStandsOneWheelOffsetToTheSideAndTheOtherWheelIsTwelveInchesFromThePollen() {
        int f = 0;                                         // left wall, center (2.634, 47.281): the wall direction is y
        double along = RED.flowerCenter(f).y;
        Pose2 left = FlowerPress.of(new Step.Intake(IntakeSource.FLOWER, RED.flowerCenter(f), ideal(f, IntakeWheel.LEFT), 3.0, 4,
                6.0, IntakeWheel.LEFT, ""), RED).press;
        Pose2 right = FlowerPress.of(new Step.Intake(IntakeSource.FLOWER, RED.flowerCenter(f), ideal(f, IntakeWheel.RIGHT), 3.0, 4,
                6.0, IntakeWheel.RIGHT, ""), RED).press;
        // facing -x (heading 180), the robot's left is -y: the LEFT wheel presses when the robot is 6.1 in toward +y
        assertEquals(along + BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN, left.y, EPS);
        assertEquals(along - BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN, right.y, EPS);
        assertEquals(left.x, right.x, EPS);
        // the wheel that is NOT pressing is 12.2 in along the wall from the POLLEN
        Point2 other = Footprint.wheelCenter(left, IntakeWheel.RIGHT);
        assertEquals(2.0 * BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN, Math.abs(other.y - along), EPS);
    }

    @Test
    public void aWheelPastTheStopLineWouldBeInsideThePressZoneButTheDerivedPressPoseIsOnTheLine() {
        // the validator's I3 rule: the pressing wheel disc against the press zone. At the press pose they just touch...
        for (int f = 0; f < RED.flowerPlates().size(); f++) {
            for (IntakeWheel w : IntakeWheel.physical()) {
                Pose2 pose = FlowerPress.poseWithWheelOnCenterLine(RED, f, w, RED.flowerPressStopLineIn());
                double gap = Footprint.wheelDisc(pose, w).signedGap(RED.flowerPressZone(f));
                assertEquals("touching, not inside", 0.0, gap, EPS);
                // ...and a wheel 0.3 in deeper is 0.3 in inside it (this is what the rule would report)
                Pose2 deeper = FlowerPress.poseWithWheelOnCenterLine(RED, f, w, RED.flowerPressStopLineIn() - 0.3);
                assertEquals(-0.3, Footprint.wheelDisc(deeper, w).signedGap(RED.flowerPressZone(f)), EPS);
                // and the plate itself IS entered (the wheel is meant to be inside it)
                assertTrue(Footprint.wheelDisc(pose, w).overlaps(RED.flowerPlate(f)));
                // the chassis does not enter the plate or even its clearance zone
                assertTrue(Footprint.chassis(pose, REACH).signedGap(RED.flowerChassisKeepOut(f)) >= -EPS);
            }
        }
    }

    // ---- a perfectly written FLOWER intake is clean ----------------------------------------------------------

    @Test
    public void theIdealLineUpIsCleanForEveryOwnFlowerAndEitherWheel() {
        for (int f : OWN_FLOWERS) {
            for (IntakeWheel w : IntakeWheel.physical()) {
                AutoRoute r = lineUpRoute(f, w, ideal(f, w));
                List<Issue> l = intakeIssues(r);
                assertTrue("FLOWER " + (f + 1) + " with the " + w + " wheel should be clean but got " + l, l.isEmpty());
            }
        }
    }

    @Test
    public void theShortBuilderFormWorksOutTheLineUpPoseForYou() {
        for (int f : OWN_FLOWERS) {
            for (IntakeWheel w : IntakeWheel.physical()) {
                Point2 c = RED.flowerCenter(f);
                AutoRoute r = base().driveConstant(ideal(f, w).x, ideal(f, w).y, ideal(f, w).headingDegrees())
                        .intakeFlower(c.x, c.y, w, 3).build();
                Step.Intake step = (Step.Intake) r.step(INTAKE);
                assertEquals(w, step.wheel());
                assertEquals(ideal(f, w).x, step.robotPose().x, EPS);
                assertEquals(ideal(f, w).y, step.robotPose().y, EPS);
                assertTrue(intakeIssues(r).isEmpty());
            }
        }
        // the six-number form names no wheel: it is the one whose middle is nearer the FLOWER's center line
        Pose2 l = ideal(0, IntakeWheel.RIGHT);
        AutoRoute r = base().driveConstant(l.x, l.y, l.headingDegrees()).intakeFlower(2.4, 48, l.x, l.y, l.headingDegrees(), 3).build();
        assertEquals(IntakeWheel.RIGHT, ((Step.Intake) r.step(INTAKE)).wheel());
    }

    @Test
    public void theWheelNamedByTheStepSurvivesTheHalfTurnForBlue() {
        AutoRoute red = lineUpRoute(0, IntakeWheel.RIGHT, ideal(0, IntakeWheel.RIGHT));
        Step.Intake blue = (Step.Intake) red.forAlliance(Alliance.BLUE).step(INTAKE);
        assertEquals("the robot's right stays the robot's right when the field turns", IntakeWheel.RIGHT, blue.wheel());
        assertTrue(blue.describe(), blue.describe().contains("RIGHT"));
        // and the blue route presses its own (right wall) FLOWER cleanly
        AllianceField b = new AllianceField(new FieldModel(), Alliance.BLUE);
        FlowerPress p = FlowerPress.of(blue, b);
        assertNotNull(p);
        assertEquals(0.0, p.lateralIn, EPS);
        assertEquals(0.0, p.headingErrorDeg(), EPS);
    }

    @Test
    public void aFlowerIntakeMustNameItsWheelAndOtherIntakesHaveNone() {
        try {
            new Step.Intake(IntakeSource.FLOWER, new Point2(2.4, 48), Pose2.ofDegrees(20, 48, 180), 3.0, 4, 6.0, null, "");
            fail("a FLOWER INTAKE without a wheel must be refused");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("wheel"));
        }
        AutoRoute garden = AutoRoute.builder("g").start(35, FRONT, -90).shootUp(4).intakeGarden(5.9, 1.4, 9, FRONT + 0.8, -90, 4).build();
        assertNull(((Step.Intake) garden.step(2)).wheel());
        assertNull(FlowerPress.of((Step.Intake) garden.step(2), RED));
    }

    // ---- I4: heading, lateral, approach length, retreat ----------------------------------------------------

    /**
     * The pose with its heading crooked by {@code deltaRad} and NOTHING ELSE wrong: it is as far from the wall as before (the
     * final approach keeps its length) and the middle of the pressing wheel is still on the FLOWER's center line. Turning the
     * robot about its pose point (the center of rotation) swings the wheel sideways, because the wheel is 8.4 in ahead of that
     * point and 6.1 in to the side: 8 degrees moves it 1.2 in along the wall, more than the 1.0 in alignment tolerance. So the
     * turned robot is slid back along the wall until the wheel is on the center line again.
     */
    private static Pose2 crookedButOtherwiseIdeal(Pose2 pose, IntakeWheel wheel, double deltaRad) {
        Pose2 turned = new Pose2(pose.x, pose.y, pose.heading + deltaRad);
        Point2 n = RED.flowerInward(0);
        double off = RED.flowerLateral(0, Footprint.wheelCenter(turned, wheel));
        return new Pose2(turned.x + n.y * off, turned.y - n.x * off, turned.heading);
    }

    @Test
    public void aCrookedLineUpIsAnErrorAndASlightlyCrookedOneIsNot() {
        Pose2 ideal = ideal(0, IntakeWheel.LEFT);
        double tol = BioBuzzConstants.Auto.FLOWER_HEADING_TOLERANCE_DEG;
        AutoRoute crooked = lineUpRoute(0, IntakeWheel.LEFT, crookedButOtherwiseIdeal(ideal, IntakeWheel.LEFT, Math.toRadians(tol + 3.0)));
        List<Issue> l = intakeIssues(crooked);
        assertEquals(l.toString(), 1, count(l, Rule.FLOWER_PRESS, Severity.ERROR));
        assertTrue(first(l, Rule.FLOWER_PRESS, Severity.ERROR).message.contains("perpendicular to the wall"));
        AutoRoute slight = lineUpRoute(0, IntakeWheel.LEFT, crookedButOtherwiseIdeal(ideal, IntakeWheel.LEFT, Math.toRadians(tol - 1.0)));
        assertTrue(intakeIssues(slight).toString(), intakeIssues(slight).isEmpty());
        // a robot that faces AWAY from the wall is as wrong as it gets
        AutoRoute away = lineUpRoute(0, IntakeWheel.LEFT, new Pose2(ideal.x, ideal.y, ideal.heading + Math.PI));
        assertTrue(count(intakeIssues(away), Rule.FLOWER_PRESS, Severity.ERROR) >= 1);
    }

    @Test
    public void aWheelThatIsNotOnTheCenterLineIsAnErrorWithinTheToleranceItIsNot() {
        Pose2 ideal = ideal(0, IntakeWheel.LEFT);            // left wall: the wall direction is y
        double tol = BioBuzzConstants.Auto.FLOWER_ALIGN_TOLERANCE_IN;
        List<Issue> off = intakeIssues(lineUpRoute(0, IntakeWheel.LEFT, new Pose2(ideal.x, ideal.y + tol + 0.7, ideal.heading)));
        assertEquals(off.toString(), 1, count(off, Rule.FLOWER_PRESS, Severity.ERROR));
        assertTrue(first(off, Rule.FLOWER_PRESS, Severity.ERROR).message.contains("center line"));
        assertTrue(intakeIssues(lineUpRoute(0, IntakeWheel.LEFT, new Pose2(ideal.x, ideal.y + tol - 0.2, ideal.heading))).isEmpty());
        assertTrue(intakeIssues(lineUpRoute(0, IntakeWheel.LEFT, new Pose2(ideal.x, ideal.y - tol + 0.2, ideal.heading))).isEmpty());
    }

    @Test
    public void standingCenteredOnTheFlowerLikeAnOldRouteIsAnErrorBecauseNeitherWheelIsOnThePollen() {
        // the old idea: the robot centered on the FLOWER with the intake in front of it. The two wheels are 6.1 in either
        // side of the center line, so neither is on the POLLEN.
        Pose2 centered = FlowerPress.poseWithWheelOnCenterLine(RED, 0, IntakeWheel.LEFT, RED.flowerPressStopLineIn()
                + BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN);
        Pose2 middle = new Pose2(centered.x, RED.flowerCenter(0).y, centered.heading);
        for (IntakeWheel w : IntakeWheel.physical()) {
            List<Issue> l = intakeIssues(lineUpRoute(0, w, middle));
            assertTrue(w + ": " + l, count(l, Rule.FLOWER_PRESS, Severity.ERROR) >= 1);
            assertEquals(BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN,
                    Math.abs(FlowerPress.of((Step.Intake) lineUpRoute(0, w, middle).step(INTAKE), RED).lateralIn), EPS);
        }
    }

    @Test
    public void namingTheWrongWheelIsAnAlignmentError() {
        // the robot stands where the LEFT wheel would press, but the step says RIGHT: the right wheel is 12.2 in away
        // from the center line
        Pose2 forLeft = ideal(0, IntakeWheel.LEFT);
        List<Issue> l = intakeIssues(lineUpRoute(0, IntakeWheel.RIGHT, forLeft));
        Issue err = first(l, Rule.FLOWER_PRESS, Severity.ERROR);
        assertTrue(err.message, err.message.contains("RIGHT wheel lines up 12.2 in"));
    }

    @Test
    public void theWheelThatDoesNotPressStaysOutsideThePlateAndItsMargin() {
        // I3: only ONE wheel may enter the plate. With the wheels 12.2 in apart the other one is nowhere near it at the press
        // pose; the validator keeps checking it (every leg of the press-in) so a robot with its wheels closer together
        // could not press with one while the other pokes into the plate
        for (int f = 0; f < RED.flowerPlates().size(); f++) {
            for (IntakeWheel w : IntakeWheel.physical()) {
                Pose2 pose = FlowerPress.poseWithWheelOnCenterLine(RED, f, w, RED.flowerPressStopLineIn());
                double gap = Footprint.wheelDisc(pose, w.other()).signedGap(RED.flowerKeepOut(f));
                assertTrue("FLOWER " + (f + 1) + ", " + w + " presses, the other wheel is " + gap + " in from the plate keep-out", gap > 5.0);
            }
        }
    }

    @Test
    public void aTooShortFinalApproachIsAnError() {
        int f = 0;
        double want = BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN;
        Pose2 close = FlowerPress.poseWithWheelOnCenterLine(RED, f, IntakeWheel.LEFT, RED.flowerPressStopLineIn() + want - 3.0);
        List<Issue> l = intakeIssues(lineUpRoute(f, IntakeWheel.LEFT, close));
        assertEquals(l.toString(), 1, count(l, Rule.FLOWER_PRESS, Severity.ERROR));
        assertTrue(first(l, Rule.FLOWER_PRESS, Severity.ERROR).message.contains("straight final approach is only 3.0 in"));
        // exactly the minimum is fine and so is a longer one
        for (double extra : new double[] {0.0, 4.0}) {
            Pose2 ok = FlowerPress.poseWithWheelOnCenterLine(RED, f, IntakeWheel.LEFT, RED.flowerPressStopLineIn() + want + extra);
            assertTrue(intakeIssues(lineUpRoute(f, IntakeWheel.LEFT, ok)).toString(), intakeIssues(lineUpRoute(f, IntakeWheel.LEFT, ok)).isEmpty());
        }
    }

    @Test
    public void aLineUpThatIsAlreadyInsideThePlateRunsTheChassisIntoIt() {
        // line-up with the wheel tip only 1 in from the wall: the chassis front is 3.7 in from it, inside the 5.033 in plate
        Pose2 inside = FlowerPress.poseWithWheelOnCenterLine(RED, 0, IntakeWheel.LEFT, 1.0);
        AutoRoute r = AutoRoute.builder("x").start(35, FRONT, -90).shootUp(4)
                .driveConstant(inside.x, inside.y, inside.headingDegrees())
                .intakeFlower(2.4, 48, inside.x, inside.y, inside.headingDegrees(), IntakeWheel.LEFT, 3).build();
        List<Issue> all = RouteValidator.check(r);
        boolean driveInPlate = false;
        for (Issue i : all) {
            driveInPlate |= i.rule == Rule.FLOWER_KEEPOUT && i.severity == Severity.ERROR && i.stepIndex == 2;
        }
        assertTrue("the DRIVE to a pose inside the plate is an ERROR: " + all, driveInPlate);
    }

    @Test
    public void aShortRetreatIsOnlyAWarning() {
        Pose2 l = ideal(0, IntakeWheel.LEFT);
        AutoRoute r = base().driveConstant(l.x, l.y, l.headingDegrees())
                .intakeFlower(2.4, 48, l.x, l.y, l.headingDegrees(), IntakeWheel.LEFT, 3).retreat(2.0).build();
        List<Issue> issues = intakeIssues(r);
        assertEquals(issues.toString(), 1, issues.size());
        assertEquals(Severity.WARN, issues.get(0).severity);
        assertEquals(Rule.FLOWER_PRESS, issues.get(0).rule);
        assertTrue(issues.get(0).message.contains("retreat is only 2.0 in"));
        assertEquals(2.0, press(r).retreatLengthIn, EPS);
        // a full retreat has no finding
        assertTrue(intakeIssues(base().driveConstant(l.x, l.y, l.headingDegrees())
                .intakeFlower(2.4, 48, l.x, l.y, l.headingDegrees(), IntakeWheel.LEFT, 3).build()).isEmpty());
    }

    // ---- I2 / I3: wheels too short, the whole plate is no longer exempt -------------------------------------------

    @Test
    public void wheelsThatReachTooLittleAreAnErrorThatSaysHowManyInchesAreMissing() {
        AutoRoute r = lineUpRoute(0, IntakeWheel.LEFT, ideal(0, IntakeWheel.LEFT));
        double shortReach = REACH - 0.7;
        RouteValidator shortWheels = new RouteValidator(new FieldModel(), new TimingModel(), shortReach);
        List<Issue> l = onStep(shortWheels.validate(r), INTAKE);
        Issue reach = first(l, Rule.FLOWER_PRESS, Severity.ERROR);
        assertTrue(reach.message, reach.message.contains("0.7 in missing"));
        assertTrue(reach.message, reach.message.contains("2.0 in"));
        // the chassis really would touch the plate (0.7 in inside the clearance zone), and it is reported as well
        Issue chassis = first(l, Rule.FLOWER_KEEPOUT, Severity.ERROR);
        assertTrue(chassis.message, chassis.message.contains("CHASSIS"));
        // the same route with the real reach has neither
        assertTrue(onStep(RouteValidator.check(r), INTAKE).isEmpty());
        // wheels that do not even reach the POLLEN say so
        RouteValidator stubs = new RouteValidator(new FieldModel(), new TimingModel(), 1.0);
        Issue none = first(onStep(stubs.validate(r), INTAKE), Rule.FLOWER_PRESS, Severity.ERROR);
        assertTrue(none.message, none.message.contains("does not even touch the POLLEN"));
    }

    @Test
    public void longerWheelsThanNeededAreFine() {
        AutoRoute r = lineUpRoute(0, IntakeWheel.LEFT, ideal(0, IntakeWheel.LEFT));
        RouteValidator longer = new RouteValidator(new FieldModel(), new TimingModel(), REACH + 1.0);
        assertTrue(onStep(longer.validate(r), INTAKE).isEmpty());
    }

    @Test
    public void noPartOfAFlowerPlateIsExemptForAnythingExceptThePressingWheelOnThePress() {
        // a plain DRIVE that pushes the WHOLE robot into the plate is an error, as before...
        Pose2 inside = FlowerPress.poseWithWheelOnCenterLine(RED, 0, IntakeWheel.LEFT, 1.0);
        List<Issue> drive = RouteValidator.check(base().driveConstant(inside.x, inside.y, inside.headingDegrees()).build());
        assertTrue(drive.toString(), count(drive, Rule.FLOWER_KEEPOUT, Severity.ERROR) >= 1);
        // ...and so is a DRIVE that merely puts the wheel into the plate: the wheel may enter only on a press-in
        Pose2 tipInside = FlowerPress.poseWithWheelOnCenterLine(RED, 0, IntakeWheel.LEFT, RED.flowerPressStopLineIn());
        List<Issue> tip = RouteValidator.check(base().driveConstant(tipInside.x, tipInside.y, tipInside.headingDegrees()).build());
        assertTrue(tip.toString(), count(tip, Rule.FLOWER_KEEPOUT, Severity.ERROR) >= 1);
        // the same pose reached by the INTAKE step's own press-in is allowed (see the clean tests above)
        AutoRoute press = lineUpRoute(0, IntakeWheel.LEFT, ideal(0, IntakeWheel.LEFT));
        assertEquals(0, count(RouteValidator.check(press), Rule.FLOWER_KEEPOUT, Severity.ERROR));
    }

    @Test
    public void thePressingWheelMayNotEnterAnotherFlowersPlate() {
        // pressing FLOWER 1 must not be an excuse to enter FLOWER 2: the exemption is for the targeted plate only
        Pose2 tipInsideOther = FlowerPress.poseWithWheelOnCenterLine(RED, 1, IntakeWheel.LEFT, RED.flowerPressStopLineIn());
        AutoRoute r = base().driveConstant(tipInsideOther.x, tipInsideOther.y, tipInsideOther.headingDegrees())
                .intakeFlower(2.4, 48, IntakeWheel.LEFT, 3).build();
        List<Issue> drive = onStep(RouteValidator.check(r), 2);
        assertTrue(drive.toString(), count(drive, Rule.FLOWER_KEEPOUT, Severity.ERROR) >= 1);
    }

    // ---- the pressed FLOWER ends the step at the retreat pose; the time includes all three phases -------------

    @Test
    public void theIntakeStepEndsAtTheRetreatPoseAndCostsApproachDwellAndRetreat() {
        AutoRoute r = lineUpRoute(0, IntakeWheel.LEFT, ideal(0, IntakeWheel.LEFT));
        RouteAnalysis a = r.analyze();
        FlowerPress p = press(r);
        RouteAnalysis.StepInfo info = a.step(INTAKE);
        assertEquals("the step ends backed out of the FLOWER", p.retreat.x, info.to.x, EPS);
        assertEquals(p.retreat.y, info.to.y, EPS);
        TimingModel t = new TimingModel();
        double slow = BioBuzzConstants.Auto.FLOWER_APPROACH_SPEED_FRACTION;
        double approach = t.legSeconds(p.approach, slow);
        double retreat = t.legSeconds(p.retreatPath, slow);
        double dwell = ((Step.Intake) r.step(INTAKE)).dwellSec();
        assertTrue("the slow legs take time", approach > 0.0 && retreat > 0.0);
        assertEquals(approach + dwell + retreat, t.flowerSeconds((Step.Intake) r.step(INTAKE), p), EPS);
        assertEquals("the drive to the line-up pose was already done, so the step costs exactly the press-in",
                t.flowerSeconds((Step.Intake) r.step(INTAKE), p), info.durationSec, EPS);
        assertTrue("longer than the dwell alone", info.durationSec > dwell + approach);
        // slower than driving the same distance at full speed
        assertTrue(approach > t.legSeconds(p.approach, 1.0));
    }

    @Test
    public void theRouteAfterAFlowerStartsFromTheRetreatPose() {
        Pose2 l = ideal(0, IntakeWheel.LEFT);
        AutoRoute r = base().driveConstant(l.x, l.y, l.headingDegrees())
                .intakeFlower(2.4, 48, l.x, l.y, l.headingDegrees(), IntakeWheel.LEFT, 3)
                .driveConstant(30, 70, 180).build();
        RouteAnalysis a = r.analyze();
        assertEquals(press(r).retreat.x, a.step(INTAKE + 1).from.x, EPS);
        assertEquals(press(r).retreat.y, a.step(INTAKE + 1).from.y, EPS);
    }

    @Test
    public void onlyAFlowerIntakeIsAPressIn() {
        AutoRoute garden = AutoRoute.builder("g").start(35, FRONT, -90).shootUp(4)
                .intakeGarden(5.9, 1.4, 9, FRONT + 0.8, -90, 4).build();
        assertNull(FlowerPress.of((Step.Intake) garden.step(2), RED));
        // a FLOWER intake whose target is near no FLOWER has nothing to press
        AutoRoute nothing = AutoRoute.builder("n").start(35, FRONT, -90).shootUp(4)
                .intakeFlower(60, 30, 30, 30, 0, IntakeWheel.LEFT, 1).build();
        assertNull(FlowerPress.of((Step.Intake) nothing.step(2), RED));
        assertFalse(RouteValidator.check(nothing).isEmpty());
    }
}
