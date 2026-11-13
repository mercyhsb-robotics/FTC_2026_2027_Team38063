package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The robot may TOUCH a field wall but never clip into or past it. Every test is a tiny route; the helper validates it
 * for RED and for its BLUE half turn and demands identical findings, so each red wall test also covers the opposite
 * wall (left/right, audience/rear) and each red corner test covers the opposite corner.
 *
 * <p>Numbers used below: the robot is a 17.80 in square, so its half-width is 8.9 in and its half-diagonal is
 * sqrt(2) * 8.9 = 12.59 in. A robot facing a wall with its center 8.9 in away just touches it. A square turning in
 * place sticks a corner out to 12.59 in from its center (at 45 degrees to the wall), so it needs a center that is
 * at least 12.59 in from a wall to turn freely.
 */
public class WallSafetyTest {
    private static final double HALF = BioBuzzConstants.Robot.WIDTH_IN / 2.0;
    private static final double HALF_DIAGONAL = Math.hypot(BioBuzzConstants.Robot.LENGTH_IN / 2.0, HALF);
    private static final double TOL = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;

    // ---- helpers -------------------------------------------------------------------------------------

    /** Validates for RED and BLUE, checks both give the same findings, returns the RED ones. */
    private static List<Issue> check(AutoRoute.Builder b) {
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

    private static int wallErrors(List<Issue> issues) {
        int n = 0;
        for (Issue i : issues) {
            if (i.rule == Rule.FIELD_BOUNDS && i.severity == Severity.ERROR) {
                n++;
            }
        }
        return n;
    }

    private static Issue wallError(List<Issue> issues) {
        for (Issue i : issues) {
            if (i.rule == Rule.FIELD_BOUNDS) {
                return i;
            }
        }
        throw new AssertionError("no FIELD_BOUNDS finding in " + issues);
    }

    private static void assertNoWallError(String why, List<Issue> issues) {
        assertEquals(why + ": " + issues, 0, wallErrors(issues));
    }

    private static void assertWallError(String why, List<Issue> issues) {
        assertTrue(why + ": expected a wall ERROR but got " + issues, wallErrors(issues) >= 1);
    }

    private static String inches(double v) {
        return String.format(Locale.ROOT, "%.1f in", v);
    }

    // ---- 1. the allowance is gone: wall-contact legs get no extra room ----------------------------------------

    @Test
    public void aLegMarkedWallContactMayTouchTheWallButNotPushIntoIt() {
        // front edge exactly on the audience wall: legal, marked or not
        assertNoWallError("touching", check(AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(30, HALF, -90).wallContact()));
        // 0.1 in of overlap is measurement noise (FIELD_BOUNDS_TOLERANCE_IN) ...
        assertNoWallError("within the tolerance", check(AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(30, HALF - TOL, -90).wallContact()));
        // ... but 0.9 in (the old 1.0 in allowance would have let this through) is an ERROR, marked or not
        List<Issue> pushed = check(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, 8.0, -90).wallContact());
        assertEquals(1, wallErrors(pushed));
        assertTrue(wallError(pushed).message, wallError(pushed).message.contains("0.9 in"));
        assertEquals(1, wallError(pushed).stepIndex);
        assertWallError("unmarked", check(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, 8.0, -90)));
        // just past the tolerance
        assertWallError("0.2 in", check(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, HALF - 0.2, -90)));
    }

    @Test
    public void theConstantThatAllowedItDoesNotExistAnyMore() {
        for (java.lang.reflect.Field f : BioBuzzConstants.Auto.class.getFields()) {
            assertTrue("a wall allowance must not come back: " + f.getName(), !f.getName().contains("WALL_CONTACT_ALLOWANCE"));
        }
    }

    // ---- 2. every wall -----------------------------------------------------------------------------------------

    @Test
    public void theAudienceAndRearWallsAreBothSolid() {
        // audience wall (blue: the rear wall)
        assertNoWallError("touching", check(AutoRoute.builder("x").start(58, 8.9, -90).driveConstant(30, HALF, -90)));
        assertWallError("0.5 in in", check(AutoRoute.builder("x").start(58, 8.9, -90).driveConstant(30, HALF - 0.5, -90)));
        // rear wall (blue: the audience wall): the legal C6 start, nudged sideways along the wall and then into it
        assertNoWallError("touching", check(AutoRoute.builder("x").start(61.5, 135.1, 90).driveConstant(60.5, 135.1, 90)));
        assertWallError("0.5 in in", check(AutoRoute.builder("x").start(61.5, 135.1, 90).driveConstant(60.5, 135.6, 90)));
    }

    @Test
    public void theLeftAndRightWallsAreBothSolid() {
        // left (red) wall (blue: the right wall)
        assertNoWallError("touching", check(AutoRoute.builder("x").start(8.9, 61.5, 180).driveConstant(8.9, 70, 180)));
        assertWallError("0.5 in in", check(AutoRoute.builder("x").start(8.9, 61.5, 180).driveConstant(8.4, 70, 180)));
        // sideways to the wall: the robot's left or right side is the part that touches
        assertNoWallError("side touching", check(AutoRoute.builder("x").start(8.9, 61.5, -90).driveConstant(8.9, 70, -90)));
        assertWallError("side 0.5 in in", check(AutoRoute.builder("x").start(8.9, 61.5, -90).driveConstant(8.4, 70, -90)));
    }

    @Test
    public void aStartPokingIntoAWallIsAnErrorAndTouchingIsNot() {
        List<Issue> in = check(AutoRoute.builder("x").start(35, 8.6, -90).waitSeconds(1));
        assertEquals(1, in.size());
        assertEquals(Rule.START_G304, in.get(0).rule);
        assertTrue(in.get(0).message, in.get(0).message.contains("out of the field by 0.3 in"));
        assertTrue(check(AutoRoute.builder("x").start(35, HALF, -90).waitSeconds(1)).isEmpty());
    }

    // ---- 3. the four corners -----------------------------------------------------------------------------------

    @Test
    public void theAudienceLeftCornerAndItsOppositeCornerAreSolid() {
        // slide along the audience wall into the corner: front edge on y = 0 and left side on x = 0
        assertNoWallError("touching both walls", check(AutoRoute.builder("x").start(58, 8.9, -90)
                .driveConstant(HALF, HALF, -90)));
        assertWallError("0.5 in into the left wall", check(AutoRoute.builder("x").start(58, 8.9, -90)
                .driveConstant(HALF - 0.5, HALF, -90)));
        assertWallError("0.5 in into the audience wall", check(AutoRoute.builder("x").start(58, 8.9, -90)
                .driveConstant(HALF, HALF - 0.5, -90)));
        // a robot wedged in the corner cannot turn: its corner would go through the walls
        List<Issue> turn = check(AutoRoute.builder("x").start(58, 8.9, -90)
                .driveConstant(HALF, HALF, -90).driveConstant(HALF, HALF, -80));
        assertWallError("turning in the corner", turn);
        assertEquals("only the turn (step 3), not the slide", 2, wallError(turn).stepIndex);
    }

    @Test
    public void theRearLeftCornerAndItsOppositeCornerAreSolid() {
        // up the left wall into the corner: left side on x = 0, front on y = 144 (heading +y)
        assertNoWallError("touching both walls", check(AutoRoute.builder("x").start(8.9, 61.5, 90)
                .driveConstant(HALF, 144 - HALF, 90)));
        assertWallError("0.5 in into the rear wall", check(AutoRoute.builder("x").start(8.9, 61.5, 90)
                .driveConstant(HALF, 144 - HALF + 0.5, 90)));
        assertWallError("0.5 in into the left wall", check(AutoRoute.builder("x").start(8.9, 61.5, 90)
                .driveConstant(HALF - 0.5, 144 - HALF, 90)));
        assertWallError("turning in the corner", check(AutoRoute.builder("x").start(8.9, 61.5, 90)
                .driveConstant(HALF, 144 - HALF, 90).driveConstant(HALF, 144 - HALF, 100)));
    }

    // ---- 4. turning in place: the corners of the rectangle, not a circle ---------------------------------------

    @Test
    public void turningInPlaceNineInchesFromTheWallPokesACornerThrough() {
        // center 9.0 in from the audience wall, facing it (front edge 0.1 in away: a legal start), then turn to 45 degrees
        // to the wall: a corner reaches 12.59 in from the center, 3.59 in through the wall
        List<Issue> l = check(AutoRoute.builder("x").start(58, 9.0, -90).driveConstant(58, 9.0, -45));
        assertEquals(1, wallErrors(l));
        assertEquals(1, wallError(l).stepIndex);
        assertTrue(wallError(l).message, wallError(l).message.contains(inches(HALF_DIAGONAL - 9.0)));
    }

    @Test
    public void theSameTurn12Point7InchesFromTheWallIsFine() {
        // 12.7 in is just more than the half-diagonal (12.59 in): the corner stops 0.11 in short of the wall
        assertNoWallError("12.7 in", check(AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(58, 12.7, -90).driveConstant(58, 12.7, -45)));
        assertNoWallError("and on to -90 -> -135", check(AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(58, 12.7, -90).driveConstant(58, 12.7, -135)));
        // but a wall 12.5 in away is too close: the corner would be 0.09 in through (inside the 0.1 tolerance) ...
        assertNoWallError("12.5 in is inside the noise tolerance", check(AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(58, 12.5, -90).driveConstant(58, 12.5, -45)));
        // ... and 12.3 in is not
        assertWallError("12.3 in", check(AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(58, 12.3, -90).driveConstant(58, 12.3, -45)));
    }

    @Test
    public void aTurnIsJudgedByTheRectangleNotByACircleAroundIt() {
        // center 11.0 in from the wall. A circle of the half-diagonal (12.59 in) would call even a tiny turn a collision.
        // The real square reaches 8.9 * (cos 10 + sin 10) = 10.31 in toward the wall after a 10 degree turn: still clear.
        assertNoWallError("10 degrees at 11 in", check(AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(58, 11.0, -90).driveConstant(58, 11.0, -100)));
        // 45 degrees reaches 12.59 in: 1.59 in through the wall
        assertWallError("45 degrees at 11 in", check(AutoRoute.builder("x").start(58, 9.2, -90)
                .driveConstant(58, 11.0, -90).driveConstant(58, 11.0, -45)));
    }

    @Test
    public void aTurnWhileSlidingAlongAWallPokesACornerThroughToo() {
        // slide along the audience wall 9.2 in from it while the heading turns evenly from -90 to -45 degrees
        List<Issue> bad = check(AutoRoute.builder("x").start(58, 9.2, -90).driveLinear(30, 9.2, -45));
        assertEquals(1, wallErrors(bad));
        // ... the corner already pokes through after the first couple of degrees, long before the end of the slide
        java.util.regex.Matcher first = java.util.regex.Pattern.compile("first at \\((\\d+\\.\\d),").matcher(wallError(bad).message);
        assertTrue(wallError(bad).message, first.find());
        assertTrue("first violation near the start of the 28 in slide: " + wallError(bad).message,
                Double.parseDouble(first.group(1)) > 54.0);
        // the same slide 12.7 in out is clean, and so is a turn the other way
        assertNoWallError("12.7 in", check(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(58, 12.7, -90)
                .driveLinear(30, 12.7, -45)));
        assertNoWallError("12.7 in, other way", check(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(58, 12.7, -90)
                .driveLinear(30, 12.7, -135)));
    }

    // ---- 5. every kind of step ---------------------------------------------------------------------------------

    @Test
    public void aGardenIntakeAtTheWallMayTouchItButNotPushIntoIt() {
        AutoRoute.Builder touching = AutoRoute.builder("x").start(35, 9.2, -90).shootUp(4)
                .driveConstant(HALF, 18, -90).driveConstant(HALF, HALF, -90).wallContact()
                .intakeGarden(5.9, 1.4, HALF, HALF, -90, 4);
        assertNoWallError("touching two walls", check(touching));
        AutoRoute.Builder pushing = AutoRoute.builder("x").start(35, 9.2, -90).shootUp(4)
                .driveConstant(HALF, 18, -90).driveConstant(HALF, 8.0, -90).wallContact()
                .intakeGarden(5.9, 1.4, HALF, 8.0, -90, 4);
        List<Issue> l = check(pushing);
        assertWallError("pushing into the audience wall", l);
        // the intake itself stands where the drive ended: the leg that got there is what moves through the wall
        assertEquals(3, wallError(l).stepIndex);
        // an intake with a pose of its own is judged on its own step (the short move to the pose pokes through)
        List<Issue> own = check(AutoRoute.builder("x").start(35, 9.2, -90).shootUp(4)
                .driveConstant(HALF, 18, -90).intakeGarden(5.9, 1.4, HALF, 8.0, -90, 4));
        assertWallError("the intake pose is in the wall", own);
        assertEquals(3, wallError(own).stepIndex);
    }

    @Test
    public void aParkPoseThatPokesThroughTheWallIsAnError() {
        // the PARK pose is 4 in left of where the DRIVE ended: that short move pushes the footprint through the left wall
        List<Issue> l = check(AutoRoute.builder("x").start(35, 9.2, -90)
                .driveReverseTangent(34, 50).driveReverseTangent(30, 90).driveLinear(12, 108, 0).park(8.0, 108, 0));
        assertEquals(1, wallErrors(l));
        assertEquals("the PARK step", 4, wallError(l).stepIndex);
        assertNoWallError("the same park, 0.1 in tolerance", check(AutoRoute.builder("x").start(35, 9.2, -90)
                .driveReverseTangent(34, 50).driveReverseTangent(30, 90).driveLinear(12, 108, 0).park(HALF, 108, 0)));
    }

    @Test
    public void theEndPoseOfARouteIsCheckedToo() {
        // the route ends on a DRIVE whose last pose is 0.7 in through the left wall
        List<Issue> l = check(AutoRoute.builder("x").start(35, 9.2, -90)
                .driveReverseTangent(34, 50).driveReverseTangent(30, 90).driveConstant(8.2, 108, 0));
        assertEquals(1, wallErrors(l));
        assertEquals(3, wallError(l).stepIndex);
        assertTrue(wallError(l).message, wallError(l).message.contains("0.7 in"));
    }

    @Test
    public void everyFindingNamesTheWallRuleAndHowFarThroughTheRobotWent() {
        List<Issue> l = check(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(30, 8.0, -90));
        Issue i = wallError(l);
        assertEquals(Severity.ERROR, i.severity);
        assertTrue(i.message, i.message.contains("leaves the field by up to 0.9 in"));
        assertTrue(i.message, i.message.contains("cannot drive or turn through a wall"));
    }
}
