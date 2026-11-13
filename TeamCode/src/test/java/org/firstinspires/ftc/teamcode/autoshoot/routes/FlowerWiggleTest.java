package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;

/**
 * The geometry of the FLOWER WIGGLE, checked the slow way: every claim of {@link FlowerWiggle} (the turn angle, the standoff, the reach) is compared with
 * a brute-force search that turns the robot a hundredth of a degree at a time and measures the real footprint at every pose. Nothing here trusts the
 * closed-form maths it is checking.
 *
 * <p>The wall frame is used throughout (the wall is x = 0, x is the distance from the wall, y runs along it, the POLLEN's middle is at (2.634, 0), a heading of
 * 180 degrees faces the wall), exactly as {@link FlowerWiggle} does.
 *
 * <p>Two robots are used. {@link #LONG} has flat wheels that stick out 4 in, so the wiggle works; {@link #REAL} is the robot of the goBILDA drawing (2.7 in),
 * for which the wiggle does NOT reach the ball once the chassis is stood far enough out to clear the plate through the swing: the finding the validator is
 * meant to give.
 */
public class FlowerWiggleTest {
    private static final double EPS = 1e-6;
    private static final FlowerWiggle.Spec LONG = WiggleTestRobot.spec();
    private static final FlowerWiggle.Spec REAL = FlowerWiggle.Spec.fromConstants();
    /** The least wheel reach (with the shipped clearance) for which the chassis, not the stop line, decides the standoff: 3.5 in works, 3.4 in does not. */
    private static final FlowerWiggle.Spec JUST_ENOUGH = FlowerWiggle.Spec.fromConstants().withIntakeReach(3.5);
    private static final Point2 BALL = new Point2(BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN, 0.0);
    private static final double HALF_PLATE = BioBuzzConstants.Field.FLOWER_PLATE_ALONG_WALL_IN / 2.0;
    private static final double PRESS = BioBuzzConstants.Auto.FLOWER_WHEEL_PRESS_OFFSET_IN;

    // ---- helpers ---------------------------------------------------------------------------------------------------

    /** The wall-frame pose of the robot after turning {@code deg} degrees (counter-clockwise positive) from the start of the swing. */
    private static Pose2 turned(FlowerWiggle.Solution s, double deg) {
        return new Pose2(s.standoffIn, s.lateralIn, Math.PI + Math.toRadians(deg));
    }

    private static double wheelToBall(FlowerWiggle.Solution s, double deg) {
        return Footprint.wheelCenter(turned(s, deg), s.wheel).distanceTo(BALL);
    }

    private static Rect chassisKeepOut(FlowerWiggle.Spec spec) {
        double c = spec.chassisClearanceIn;
        return new Rect(-c, BioBuzzConstants.Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN + c, -HALF_PLATE - c, HALF_PLATE + c);
    }

    private static Rect stopZone() {
        return new Rect(0.0, BALL.x, -HALF_PLATE, HALF_PLATE);
    }

    // ---- the turn angle ----------------------------------------------------------------------------------------------

    @Test
    public void theTurnIsWhereTheWheelCircleFirstMeetsThePressDistanceFoundByBruteForce() {
        for (IntakeWheel w : IntakeWheel.physical()) {
            FlowerWiggle.Solution s = FlowerWiggle.solve(w, LONG);
            assertTrue(w + " reaches", s.reaches);
            // the wheel's middle really is the press offset from the ball's middle at the derived turn...
            assertEquals(w + " distance at the turn", PRESS, wheelToBall(s, s.turnDeg()), 1e-9);
            // ...and the FIRST turn at which it is, going the way the solution turns, one hundredth of a degree at a time
            double dir = Math.signum(s.turnRad);
            double first = Double.NaN;
            for (int k = 0; k <= 4000; k++) {
                double deg = dir * k * 0.01;
                if (wheelToBall(s, deg) <= PRESS) {
                    first = deg;
                    break;
                }
            }
            assertEquals(w + " first meeting", s.turnDeg(), first, 0.0101);
            // nothing earlier is closer than the press offset: the wheel never pushes harder than asked
            for (double deg = 0.0; Math.abs(deg) < Math.abs(s.turnDeg()) - 0.02; deg += dir * 0.01) {
                assertTrue(w + " " + deg, wheelToBall(s, deg) > PRESS - 1e-9);
            }
        }
    }

    @Test
    public void theTurnCarriesTheWheelTowardTheBallAndMirrorsForTheOtherWheel() {
        FlowerWiggle.Solution left = FlowerWiggle.solve(IntakeWheel.LEFT, LONG);
        FlowerWiggle.Solution right = FlowerWiggle.solve(IntakeWheel.RIGHT, LONG);
        // facing the wall with the ball on the inboard side: the left wheel is carried toward the ball by turning clockwise, the right wheel counter-clockwise
        assertTrue("left turns clockwise", left.turnRad < 0.0);
        assertTrue("right turns counter-clockwise", right.turnRad > 0.0);
        for (FlowerWiggle.Solution s : new FlowerWiggle.Solution[] {left, right}) {
            double last = Double.POSITIVE_INFINITY;
            for (int i = 0; i <= 20; i++) {
                double d = wheelToBall(s, s.turnDeg() * i / 20.0);
                assertTrue(s.wheel + " gets closer at every step, " + d + " after " + last, d < last);
                last = d;
            }
        }
        // the two are mirror images of each other along the wall
        assertEquals(left.standoffIn, right.standoffIn, EPS);
        assertEquals(left.lateralIn, -right.lateralIn, EPS);
        assertEquals(left.turnRad, -right.turnRad, EPS);
        assertEquals(left.missingReachIn, right.missingReachIn, 1e-4);
    }

    @Test
    public void theLeverArmIsTheHypotOfTheWheelsForwardDistanceAndSidewaysOffsetAndItNeverChanges() {
        double forward = BioBuzzConstants.Robot.FRONT_EXTENT_IN - BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0;
        double lever = Math.hypot(forward, BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN);
        assertEquals(10.357, lever, 0.001);                          // "about 10.4 in" of the spec
        assertEquals(lever, FlowerWiggle.leverArmIn(), 1e-12);
        FlowerWiggle.Solution s = FlowerWiggle.solve(IntakeWheel.LEFT, LONG);
        assertEquals(lever, s.leverArmIn, 1e-12);
        // while the robot turns about its pose point the middle of the wheel stays exactly that far from it
        Point2 pose = new Point2(s.standoffIn, s.lateralIn);
        for (double deg = 0.0; deg >= s.turnDeg(); deg -= 0.5) {
            assertEquals(lever, Footprint.wheelCenter(turned(s, deg), IntakeWheel.LEFT).distanceTo(pose), 1e-9);
        }
    }

    @Test
    public void theTurnNeverExceedsTheLimitAndAMoreCrampedLimitMakesThePickUpImpossible() {
        for (IntakeWheel w : IntakeWheel.physical()) {
            assertTrue(Math.abs(FlowerWiggle.solve(w, LONG).turnDeg()) <= BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_TURN_DEG + EPS);
            assertTrue("even a pick-up that cannot reach turns no farther than the limit",
                    Math.abs(FlowerWiggle.solve(w, REAL).turnDeg()) <= BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_TURN_DEG + EPS);
        }
        FlowerWiggle.Solution cramped = FlowerWiggle.solve(IntakeWheel.LEFT, LONG.withMaxTurnDeg(5.0));
        assertFalse("a 5 degree turn cannot reach the ball", cramped.reaches);
        assertTrue("and no length of wheel helps", Double.isInfinite(cramped.missingReachIn));
        assertTrue(cramped.shortfallText(), cramped.shortfallText().contains("no length of wheel would help"));
        assertEquals(-5.0, cramped.turnDeg(), 1e-9);
    }

    // ---- where the robot starts ---------------------------------------------------------------------------------------

    @Test
    public void theWheelStartsBesideTheBallAtTheStartOffsetWithTheBallBetweenTheWheelAndTheCenterLine() {
        for (IntakeWheel w : IntakeWheel.physical()) {
            FlowerWiggle.Solution s = FlowerWiggle.solve(w, LONG);
            Point2 wheel = Footprint.wheelCenter(s.startPose(), w);
            // along the wall the wheel is the start offset from the ball's middle line
            assertEquals(w + " start offset", BioBuzzConstants.Auto.FLOWER_WHEEL_START_OFFSET_IN, Math.abs(wheel.y - BALL.y), 1e-9);
            // the ball is INBOARD: between the wheel and the robot's center line (the pose point's y), so going from the wheel to the ball and on to the center
            // line is one direction along the wall
            double wheelToBallY = BALL.y - wheel.y;
            double ballToCenterLineY = s.lateralIn - BALL.y;
            assertTrue(w + " the ball is between the wheel and the center line", wheelToBallY * ballToCenterLineY > 0.0);
            assertEquals(w + " the wheel and the center line are one wheel offset apart", BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN,
                    Math.abs(wheel.y - s.lateralIn), 1e-9);
            // the wheel passes the ball without touching it: the two discs are farther apart than the two radii
            assertTrue(w + " no contact at the start", wheel.distanceTo(BALL) > BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0
                    + BioBuzzConstants.Auto.POLLEN_DIAMETER_IN / 2.0);
            // and the robot faces the wall squarely
            assertEquals(Math.PI, Math.abs(s.startPose().heading), EPS);
        }
    }

    @Test
    public void withTheBallOutboardOfTheWheelTheStartMovesToTheOtherSideAndTheTurnReverses() {
        FlowerWiggle.Spec outboard = LONG.withBallInboard(false);
        for (IntakeWheel w : IntakeWheel.physical()) {
            FlowerWiggle.Solution in = FlowerWiggle.solve(w, LONG);
            FlowerWiggle.Solution out = FlowerWiggle.solve(w, outboard);
            Point2 wheel = Footprint.wheelCenter(out.startPose(), w);
            // the ball is now on the far side of the wheel from the center line: the wheel is between the ball and the center line
            double ballToWheel = wheel.y - BALL.y;
            double wheelToCenter = out.lateralIn - wheel.y;
            assertTrue(w + " the wheel is between the ball and the center line", ballToWheel * wheelToCenter > 0.0);
            assertEquals(BioBuzzConstants.Auto.FLOWER_WHEEL_START_OFFSET_IN, Math.abs(ballToWheel), 1e-9);
            assertTrue(w + " the turn goes the other way", in.turnRad * out.turnRad < 0.0);
            assertTrue("the pose point is farther along the wall from the ball", Math.abs(out.lateralIn) > Math.abs(in.lateralIn));
        }
    }

    // ---- the standoff includes the swing -------------------------------------------------------------------------------

    /** The smallest signed gap over a whole swing between {@code chassis(pose, reach)} and the keep-out, and between the wheel disc and the stop zone. */
    private static double[] swingGaps(FlowerWiggle.Solution s) {
        double chassis = Double.POSITIVE_INFINITY;
        double stop = Double.POSITIVE_INFINITY;
        double other = Double.POSITIVE_INFINITY;
        double wall = Double.POSITIVE_INFINITY;
        double plateKeepOut = BioBuzzConstants.Auto.FLOWER_KEEPOUT_MARGIN_IN;
        Rect plateMargin = new Rect(0.0, BioBuzzConstants.Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN, -HALF_PLATE, HALF_PLATE).inflate(plateKeepOut);
        int steps = (int) Math.ceil(Math.abs(s.turnDeg()) / 0.01);
        for (int k = 0; k <= steps; k++) {                       // a hundredth of a degree at a time, the end of the turn included
            Pose2 p = turned(s, s.turnDeg() * k / steps);
            chassis = Math.min(chassis, Footprint.chassis(p, s.spec.intakeReachIn).signedGap(chassisKeepOut(s.spec)));
            stop = Math.min(stop, Footprint.wheelDiscGap(p, s.wheel, stopZone()));
            other = Math.min(other, Footprint.wheelDiscGap(p, s.wheel.other(), plateMargin));
            wall = Math.min(wall, -Footprint.of(p).signedOvershoot(new Rect(0.0, 100.0, -100.0, 100.0)));
        }
        return new double[] {chassis, stop, other, wall};
    }

    @Test
    public void theChassisNeverComesWithinTheClearanceOfThePlateAtAnyPoseOfTheSwingAndTheOtherRulesHoldToo() {
        for (FlowerWiggle.Spec spec : new FlowerWiggle.Spec[] {LONG, JUST_ENOUGH}) {
            for (IntakeWheel w : IntakeWheel.physical()) {
                FlowerWiggle.Solution s = FlowerWiggle.solve(w, spec);
                assertTrue(w + " reaches with a wheel reach of " + spec.intakeReachIn, s.reaches);
                double[] g = swingGaps(s);
                assertTrue(w + " chassis gap to the clearance line " + g[0], g[0] >= -1e-9);
                assertTrue(w + " pressing wheel short of the stop line " + g[1], g[1] >= -1e-9);
                assertTrue(w + " other wheel off the plate " + g[2], g[2] >= -1e-9);
                assertTrue(w + " nothing through the wall " + g[3], g[3] >= -1e-9);
                // the approach and the retreat are straight back from the start along the wall's normal, so they only move AWAY from the plate
                for (double extra = 0.0; extra <= 12.0; extra += 0.5) {
                    Pose2 p = new Pose2(s.standoffIn + extra, s.lateralIn, Math.PI);
                    assertTrue(Footprint.chassis(p, spec.intakeReachIn).signedGap(chassisKeepOut(spec)) >= -1e-9);
                }
            }
        }
    }

    @Test
    public void theStandoffIsTheSmallestOneThatKeepsEveryRule() {
        // with wheels that stick out 3.5 in the CHASSIS decides: at the derived standoff it touches its clearance line at the worst moment of the swing
        FlowerWiggle.Solution s = FlowerWiggle.solve(IntakeWheel.LEFT, JUST_ENOUGH);
        double[] g = swingGaps(s);
        assertEquals("the chassis just touches its clearance line", 0.0, g[0], 1e-4);
        // a robot standing even 0.01 in closer, turning by the angle that is right FOR THAT standoff, breaks a rule
        double closer = s.standoffIn - 0.01;
        FlowerWiggle.Turn t = FlowerWiggle.turnFor(JUST_ENOUGH, IntakeWheel.LEFT, closer, s.lateralIn);
        double worst = Double.POSITIVE_INFINITY;
        int steps = (int) Math.ceil(Math.abs(Math.toDegrees(t.phi)) / 0.01);
        for (int k = 0; k <= steps; k++) {
            Pose2 p = new Pose2(closer, s.lateralIn, Math.PI + t.phi * k / steps);
            worst = Math.min(worst, Footprint.chassis(p, JUST_ENOUGH.intakeReachIn).signedGap(chassisKeepOut(JUST_ENOUGH)));
        }
        assertTrue("0.01 in closer the chassis is inside its clearance line: " + worst, worst < -1e-4);
        // with wheels that reach 4 in the wheel's STOP LINE decides instead: the wheel just touches the POLLEN's center line at the end of the push
        FlowerWiggle.Solution longer = FlowerWiggle.solve(IntakeWheel.LEFT, LONG);
        assertEquals("the pressing wheel just touches the stop line", 0.0, swingGaps(longer)[1], 1e-4);
        assertTrue("and the chassis has room to spare", swingGaps(longer)[0] > 0.1);
    }

    @Test
    public void theSwingCostsStandoffThatAStraightPressWouldNotNeed() {
        // a straight press puts the chassis front on its clearance line: plate depth + clearance + (front extent - reach); with the 2.7 in wheels of the
        // drawing that is what decides (the wheel's stop line needs only 12.404 in)
        double straight = BioBuzzConstants.Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN + BioBuzzConstants.Auto.FLOWER_CHASSIS_CLEARANCE_IN
                + BioBuzzConstants.Robot.FRONT_EXTENT_IN - REAL.intakeReachIn;
        assertEquals(12.603, straight, 1e-9);
        FlowerWiggle.Solution real = FlowerWiggle.solve(IntakeWheel.LEFT, REAL);
        assertEquals(straight, real.noSwingStandoffIn, 1e-9);
        assertTrue("the swing makes the robot stand more than two inches farther out: " + real.swingExtraStandoffIn(), real.swingExtraStandoffIn() > 2.0);
        // with wheels that reach 3.5 in the wheel's own stop line is the straight press's limit, 12.404 in, and the swing still costs more than an inch
        FlowerWiggle.Solution s = FlowerWiggle.solve(IntakeWheel.LEFT, JUST_ENOUGH);
        assertEquals(BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN + BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0
                + BioBuzzConstants.Robot.FRONT_EXTENT_IN - BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0, s.noSwingStandoffIn, 1e-9);
        assertTrue("the swing makes the robot stand more than an inch farther out: " + s.swingExtraStandoffIn(), s.swingExtraStandoffIn() > 1.0);
        assertEquals(s.standoffIn - s.noSwingStandoffIn, s.swingExtraStandoffIn(), 1e-12);
    }

    // ---- whether the wheel can still reach -----------------------------------------------------------------------------

    @Test
    public void withTheShippedNumbersTheWheelFallsShortAndTheSolutionSaysByHowManyInches() {
        for (IntakeWheel w : IntakeWheel.physical()) {
            FlowerWiggle.Solution s = FlowerWiggle.solve(w, REAL);
            assertFalse(w + " cannot reach with the 2.7 in of the drawing", s.reaches);
            assertTrue("it gets no closer than " + s.closestIn, s.closestIn > PRESS + 0.5);
            assertEquals("the missing reach is about three quarters of an inch", 0.77, s.missingReachIn, 0.01);
            // the figure is right: that much more reach works, a hair less does not
            assertTrue(w + " works with the missing reach added",
                    FlowerWiggle.solve(w, REAL.withIntakeReach(REAL.intakeReachIn + s.missingReachIn + 0.002)).reaches);
            assertFalse(w + " does not work with a hair less",
                    FlowerWiggle.solve(w, REAL.withIntakeReach(REAL.intakeReachIn + s.missingReachIn - 0.002)).reaches);
            String text = s.shortfallText();
            assertTrue(text, text.contains("0.8 in more than they do"));
            assertTrue(text, text.contains("3.5 in in front of the chassis"));
        }
    }

    @Test
    public void longerWheelsOrLessClearanceBothBringTheBallWithinReach() {
        assertTrue(FlowerWiggle.solve(IntakeWheel.LEFT, REAL.withIntakeReach(3.6)).reaches);
        assertFalse(FlowerWiggle.solve(IntakeWheel.LEFT, REAL.withIntakeReach(3.3)).reaches);
        // less clearance (0.1 in) lets the chassis stand closer, which is what the reach was lost to: now 3.1 in is enough
        assertTrue(FlowerWiggle.solve(IntakeWheel.LEFT, REAL.withChassisClearance(0.1).withIntakeReach(3.2)).reaches);
        // more clearance costs reach: the same 3.6 in wheels no longer do it with 1.5 in of clearance
        assertFalse(FlowerWiggle.solve(IntakeWheel.LEFT, REAL.withChassisClearance(1.5).withIntakeReach(3.6)).reaches);
    }

    @Test
    public void wheelsThatAreLongerThanNeededChangeNothingOnceTheStopLineDecides() {
        FlowerWiggle.Solution a = FlowerWiggle.solve(IntakeWheel.LEFT, REAL.withIntakeReach(4.0));
        FlowerWiggle.Solution b = FlowerWiggle.solve(IntakeWheel.LEFT, REAL.withIntakeReach(6.0));
        assertEquals(a.standoffIn, b.standoffIn, 1e-6);
        assertEquals(a.turnRad, b.turnRad, 1e-9);
    }

    // ---- cycles and poses ---------------------------------------------------------------------------------------------------

    /** Situation: cycles turned back into POLLEN. Expected: one POLLEN per cyclesPerPollen cycles, whole POLLEN only (the cap makes 6 cycles of a 4-POLLEN pick-up at 2 per POLLEN deliver 3). */
    @Test
    public void cyclesAreTurnedBackIntoWholePollen() {
        assertEquals(4, FlowerWiggle.pollenFor(LONG.withCycles(1, 6), 4));
        assertEquals(3, FlowerWiggle.pollenFor(LONG.withCycles(2, 6), FlowerWiggle.cyclesFor(LONG.withCycles(2, 6), 4)));
        assertEquals("one cycle of two is not a POLLEN yet", 0, FlowerWiggle.pollenFor(LONG.withCycles(2, 6), 1));
        assertEquals(4, FlowerWiggle.pollenFor(LONG.withCycles(2, 8), FlowerWiggle.cyclesFor(LONG.withCycles(2, 8), 4)));
    }

    @Test
    public void theCyclesComeFromTheCountAndAreCapped() {
        assertEquals(1, FlowerWiggle.cyclesFor(REAL, 1));
        assertEquals(4, FlowerWiggle.cyclesFor(REAL, 4));
        assertEquals("a bad count cannot swallow the match", BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_CYCLES, FlowerWiggle.cyclesFor(REAL, 40));
        FlowerWiggle.Spec twoEach = REAL.withCycles(2, 6);
        assertEquals(2, FlowerWiggle.cyclesFor(twoEach, 1));
        assertEquals(6, FlowerWiggle.cyclesFor(twoEach, 3));
        assertEquals("capped", 6, FlowerWiggle.cyclesFor(twoEach, 4));
    }

    @Test
    public void thePushedPoseIsTheStartPoseTurnedAboutThePosePointAndNeverPastStraightOnTheWayBack() {
        FlowerWiggle.Solution s = FlowerWiggle.solve(IntakeWheel.RIGHT, LONG);
        Pose2 start = s.startPose();
        Pose2 pushed = s.pushedPose();
        assertEquals(start.x, pushed.x, 0.0);
        assertEquals(start.y, pushed.y, 0.0);
        assertEquals("turned by the derived angle", s.turnRad, org.firstinspires.ftc.teamcode.autoshoot.core.Angles.error(start.heading, pushed.heading), 1e-12);
        // turning back by the same angle lands exactly on the start heading, not past it
        assertEquals(start.heading, org.firstinspires.ftc.teamcode.autoshoot.core.Angles.normalize(pushed.heading - s.turnRad), 1e-12);
    }

    @Test
    public void aSolutionIsOnlyAskedForARealWheel() {
        try {
            FlowerWiggle.solve(IntakeWheel.AUTO, LONG);
            org.junit.Assert.fail("AUTO is not a wheel");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("WheelChoice"));
        }
    }
}
