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
 * <p>Numbers used below. The robot's outline is 17.80 x 17.80 in, but its position (the pose, the point it spins about)
 * is NOT in the middle of it: the front edge is {@code F} = 9.77 in from the position, the rear edge {@code B} = 8.03 in
 * and each side edge {@code S} = 8.90 in. So a robot touches a wall with its position 9.77 in from it when it faces the
 * wall front-first, 8.03 in when it faces away, and 8.90 in when it stands sideways to it. A robot turning in place
 * sticks its farthest corner (a front one) out to {@code R} = hypot(9.77, 8.90) = 13.22 in from its position, so it
 * needs a position at least 13.22 in from a wall to turn freely.
 */
public class WallSafetyTest {
    private static final double F = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double B = BioBuzzConstants.Robot.REAR_EXTENT_IN;
    private static final double S = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
    private static final double R = Math.hypot(F, S);
    private static final double TOL = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;

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
        assertNoWallError("touching", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(30, F, -90).wallContact()));
        // 0.1 in of overlap is measurement noise (FIELD_BOUNDS_TOLERANCE_IN) ...
        assertNoWallError("within the tolerance", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(30, F - TOL, -90).wallContact()));
        // ... but 0.9 in (the old 1.0 in allowance would have let this through) is an ERROR, marked or not
        List<Issue> pushed = check(AutoRoute.builder("x").start(58, F, -90).driveConstant(30, F - 0.9, -90).wallContact());
        assertEquals(1, wallErrors(pushed));
        assertTrue(wallError(pushed).message, wallError(pushed).message.contains("0.9 in"));
        assertEquals(1, wallError(pushed).stepIndex);
        assertWallError("unmarked", check(AutoRoute.builder("x").start(58, F, -90).driveConstant(30, F - 0.9, -90)));
        // just past the tolerance
        assertWallError("0.2 in", check(AutoRoute.builder("x").start(58, F, -90).driveConstant(30, F - 0.2, -90)));
        // the same with the BACK edge to the wall: the position may be only 8.03 in from it, and 0.9 in less is an ERROR
        assertNoWallError("back touching", check(AutoRoute.builder("x").start(58, B, 90).driveConstant(30, B, 90).wallContact()));
        assertWallError("back 0.9 in in", check(AutoRoute.builder("x").start(58, B, 90).driveConstant(30, B - 0.9, 90).wallContact()));
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
        assertNoWallError("touching", check(AutoRoute.builder("x").start(58, F, -90).driveConstant(30, F, -90)));
        assertWallError("0.5 in in", check(AutoRoute.builder("x").start(58, F, -90).driveConstant(30, F - 0.5, -90)));
        // rear wall (blue: the audience wall): the legal C6 start, front edge on the wall, nudged sideways along the wall
        // and then into it
        assertNoWallError("touching", check(AutoRoute.builder("x").start(61.5, SIZE - F, 90).driveConstant(60.5, SIZE - F, 90)));
        assertWallError("0.5 in in", check(AutoRoute.builder("x").start(61.5, SIZE - F, 90).driveConstant(60.5, SIZE - F + 0.5, 90)));
        // the same walls with the back edge on them (the shooter side)
        assertNoWallError("back touching", check(AutoRoute.builder("x").start(58, B, 90).driveConstant(30, B, 90)));
        assertWallError("back 0.5 in in", check(AutoRoute.builder("x").start(58, B, 90).driveConstant(30, B - 0.5, 90)));
        assertNoWallError("rear wall, back touching", check(AutoRoute.builder("x").start(61.5, SIZE - B, -90).driveConstant(60.5, SIZE - B, -90)));
        assertWallError("rear wall, back 0.5 in in", check(AutoRoute.builder("x").start(61.5, SIZE - B, -90).driveConstant(60.5, SIZE - B + 0.5, -90)));
    }

    @Test
    public void theLeftAndRightWallsAreBothSolid() {
        // left (red) wall (blue: the right wall), front edge on it
        assertNoWallError("touching", check(AutoRoute.builder("x").start(F, 61.5, 180).driveConstant(F, 70, 180)));
        assertWallError("0.5 in in", check(AutoRoute.builder("x").start(F, 61.5, 180).driveConstant(F - 0.5, 70, 180)));
        // back edge on it
        assertNoWallError("back touching", check(AutoRoute.builder("x").start(B, 61.5, 0).driveConstant(B, 70, 0)));
        assertWallError("back 0.5 in in", check(AutoRoute.builder("x").start(B, 61.5, 0).driveConstant(B - 0.5, 70, 0)));
        // sideways to the wall: the robot's left or right side is the part that touches
        assertNoWallError("side touching", check(AutoRoute.builder("x").start(S, 61.5, -90).driveConstant(S, 70, -90)));
        assertWallError("side 0.5 in in", check(AutoRoute.builder("x").start(S, 61.5, -90).driveConstant(S - 0.5, 70, -90)));
    }

    @Test
    public void aStartPokingIntoAWallIsAnErrorAndTouchingIsNot() {
        List<Issue> in = check(AutoRoute.builder("x").start(35, F - 0.3, -90).waitSeconds(1));
        assertEquals(1, in.size());
        assertEquals(Rule.START_G304, in.get(0).rule);
        assertTrue(in.get(0).message, in.get(0).message.contains("out of the field by 0.3 in"));
        assertTrue(check(AutoRoute.builder("x").start(35, F, -90).waitSeconds(1)).isEmpty());
        // the SAME position facing away from the wall does not poke through it (the back edge is nearer to the position than the
        // front edge); it only fails the "must touch a wall" rule, because it is now 1.4 in off the wall
        List<Issue> away = check(AutoRoute.builder("x").start(35, F - 0.3, 90).waitSeconds(1));
        assertEquals(away.toString(), 1, away.size());
        assertTrue(away.get(0).message, !away.get(0).message.contains("sticks out") && away.get(0).message.contains("must touch"));
        // ... until it is closer than the back edge
        List<Issue> back = check(AutoRoute.builder("x").start(35, B - 0.3, 90).waitSeconds(1));
        assertEquals(1, back.size());
        assertTrue(back.get(0).message, back.get(0).message.contains("out of the field by 0.3 in"));
    }

    // ---- 3. the four corners -----------------------------------------------------------------------------------

    @Test
    public void theAudienceLeftCornerAndItsOppositeCornerAreSolid() {
        // slide along the audience wall into the corner: front edge on y = 0 and left side on x = 0
        assertNoWallError("touching both walls", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(S, F, -90)));
        assertWallError("0.5 in into the left wall", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(S - 0.5, F, -90)));
        assertWallError("0.5 in into the audience wall", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(S, F - 0.5, -90)));
        // a robot wedged in the corner cannot turn: its corner would go through the walls
        List<Issue> turn = check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(S, F, -90).driveConstant(S, F, -80));
        assertWallError("turning in the corner", turn);
        assertEquals("only the turn (step 3), not the slide", 2, wallError(turn).stepIndex);
    }

    @Test
    public void theRearLeftCornerAndItsOppositeCornerAreSolid() {
        // up the left wall into the corner: left side on x = 0, front on y = 144 (heading +y)
        assertNoWallError("touching both walls", check(AutoRoute.builder("x").start(S, 61.5, 90)
                .driveConstant(S, SIZE - F, 90)));
        assertWallError("0.5 in into the rear wall", check(AutoRoute.builder("x").start(S, 61.5, 90)
                .driveConstant(S, SIZE - F + 0.5, 90)));
        assertWallError("0.5 in into the left wall", check(AutoRoute.builder("x").start(S, 61.5, 90)
                .driveConstant(S - 0.5, SIZE - F, 90)));
        assertWallError("turning in the corner", check(AutoRoute.builder("x").start(S, 61.5, 90)
                .driveConstant(S, SIZE - F, 90).driveConstant(S, SIZE - F, 100)));
        // with the BACK edge in the corner (heading -y): the position is only 8.03 in from the rear wall
        assertNoWallError("back in the corner", check(AutoRoute.builder("x").start(S, 61.5, -90)
                .driveConstant(S, SIZE - B, -90)));
        assertWallError("back 0.5 in into the rear wall", check(AutoRoute.builder("x").start(S, 61.5, -90)
                .driveConstant(S, SIZE - B + 0.5, -90)));
    }

    // ---- 4. turning in place: the corners of the rectangle, not a circle ---------------------------------------

    @Test
    public void turningInPlaceNextToTheWallPokesAFrontCornerThrough() {
        // center 0.1 in more than the front edge from the audience wall, facing it (front edge 0.1 in away: a legal start), then
        // turn to 45 degrees to the wall: a front corner reaches 13.22 in from the center, 3.3 in through the wall
        List<Issue> l = check(AutoRoute.builder("x").start(58, F + 0.1, -90).driveConstant(58, F + 0.1, -45));
        assertEquals(1, wallErrors(l));
        assertEquals(1, wallError(l).stepIndex);
        assertTrue(wallError(l).message, wallError(l).message.contains(inches(R - (F + 0.1))));
    }

    @Test
    public void theSameTurnJustOutsideTheCircumscribedRadiusIsFine() {
        // R + 0.1 in is just more than the farthest corner (13.22 in): the corner stops 0.1 in short of the wall
        double y = R + 0.1;
        assertNoWallError("R + 0.1 in", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(58, y, -90).driveConstant(58, y, -45)));
        assertNoWallError("and on to -90 -> -135", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(58, y, -90).driveConstant(58, y, -135)));
        // but a wall 0.09 in too close is inside the 0.1 in noise tolerance ...
        double near = R - 0.09;
        assertNoWallError("inside the noise tolerance", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(58, near, -90).driveConstant(58, near, -45)));
        // ... and 0.3 in is not
        double tooNear = R - 0.3;
        assertWallError("0.3 in too close", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(58, tooNear, -90).driveConstant(58, tooNear, -45)));
    }

    @Test
    public void aTurnIsJudgedByTheRectangleNotByACircleAroundIt() {
        // center 11.5 in from the wall. A circle of the circumscribed radius (13.22 in) would call even a tiny turn a collision.
        // The real rectangle reaches 9.77 * cos 10 + 8.90 * sin 10 = 11.17 in toward the wall after a 10 degree turn: still clear.
        assertNoWallError("10 degrees at 11.5 in", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(58, 11.5, -90).driveConstant(58, 11.5, -100)));
        // 45 degrees reaches 13.2 in: 1.7 in through the wall
        assertWallError("45 degrees at 11.5 in", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(58, 11.5, -90).driveConstant(58, 11.5, -45)));
    }

    @Test
    public void theBackOfTheRobotTurnsCloserToAWallThanTheFront() {
        // facing away from the audience wall (back edge toward it, 8.03 in): a 10 degree turn reaches 8.03 * cos 10 + 8.90 * sin 10
        // = 9.45 in toward the wall. At 9.0 in from the wall that is 0.45 in through it, so the turn is an ERROR ...
        assertWallError("back, 10 degrees, 9.0 in out", check(AutoRoute.builder("x").start(58, B, 90)
                .driveConstant(58, 9.0, 90).driveConstant(58, 9.0, 100)));
        // ... while the same turn with the FRONT toward the wall at the same distance is not even touching it (9.0 < 9.77 would
        // be a clip, so give it 11.5 in): the front reaches 11.17 in, clear
        assertNoWallError("front, 10 degrees, 11.5 in out", check(AutoRoute.builder("x").start(58, F, -90)
                .driveConstant(58, 11.5, -90).driveConstant(58, 11.5, -100)));
        // 9.6 in out is enough for the back (9.45 in) but not for the front (11.17 in)
        assertNoWallError("back, 10 degrees, 9.6 in out", check(AutoRoute.builder("x").start(58, B, 90)
                .driveConstant(58, 9.6, 90).driveConstant(58, 9.6, 100)));
    }

    @Test
    public void aTurnWhileSlidingAlongAWallPokesACornerThroughToo() {
        // slide along the audience wall with the front edge on it while the heading turns evenly from -90 to -45 degrees
        List<Issue> bad = check(AutoRoute.builder("x").start(58, F, -90).driveLinear(30, F, -45));
        assertEquals(1, wallErrors(bad));
        // ... the corner already pokes through after the first couple of degrees, long before the end of the slide
        java.util.regex.Matcher first = java.util.regex.Pattern.compile("first at \\((\\d+\\.\\d),").matcher(wallError(bad).message);
        assertTrue(wallError(bad).message, first.find());
        assertTrue("first violation near the start of the 28 in slide: " + wallError(bad).message,
                Double.parseDouble(first.group(1)) > 54.0);
        // the same slide R + 0.1 in out is clean, and so is a turn the other way
        double y = R + 0.1;
        assertNoWallError("R + 0.1 in", check(AutoRoute.builder("x").start(58, F, -90).driveConstant(58, y, -90)
                .driveLinear(30, y, -45)));
        assertNoWallError("R + 0.1 in, other way", check(AutoRoute.builder("x").start(58, F, -90).driveConstant(58, y, -90)
                .driveLinear(30, y, -135)));
    }

    // ---- 5. every kind of step ---------------------------------------------------------------------------------

    @Test
    public void aGardenIntakeAtTheWallMayTouchItButNotPushIntoIt() {
        // facing the audience wall, front edge on it, left side on the red wall: the position is (S, F)
        AutoRoute.Builder touching = AutoRoute.builder("x").start(35, F, -90).shootUp(4)
                .driveConstant(S, 18, -90).driveConstant(S, F, -90).wallContact()
                .intakeGarden(5.9, 1.4, S, F, -90, 4);
        assertNoWallError("touching two walls", check(touching));
        AutoRoute.Builder pushing = AutoRoute.builder("x").start(35, F, -90).shootUp(4)
                .driveConstant(S, 18, -90).driveConstant(S, F - 0.9, -90).wallContact()
                .intakeGarden(5.9, 1.4, S, F - 0.9, -90, 4);
        List<Issue> l = check(pushing);
        assertWallError("pushing into the audience wall", l);
        // the intake itself stands where the drive ended: the leg that got there is what moves through the wall
        assertEquals(3, wallError(l).stepIndex);
        // an intake with a pose of its own is judged on its own step (the short move to the pose pokes through)
        List<Issue> own = check(AutoRoute.builder("x").start(35, F, -90).shootUp(4)
                .driveConstant(S, 18, -90).intakeGarden(5.9, 1.4, S, F - 0.9, -90, 4));
        assertWallError("the intake pose is in the wall", own);
        assertEquals(3, wallError(own).stepIndex);
    }

    @Test
    public void aParkPoseThatPokesThroughTheWallIsAnError() {
        // (the robot starts 0.5 in off the wall: a robot with its front exactly on the wall cannot turn toward it at all, and these
        // reverse-tangent legs turn it a degree or two.) The PARK pose is 4.9 in left of where the DRIVE ended, facing +x with the
        // BACK edge toward the red wall: the position may
        // be no closer than 8.03 in to it, so 7.13 in pushes the footprint 0.9 in through
        List<Issue> l = check(AutoRoute.builder("x").start(35, F + 0.5, -90)
                .driveReverseTangent(34, 50).driveReverseTangent(30, 90).driveLinear(12, 108, 0).park(B - 0.9, 108, 0));
        assertEquals(1, wallErrors(l));
        assertEquals("the PARK step", 4, wallError(l).stepIndex);
        assertNoWallError("the same park, touching", check(AutoRoute.builder("x").start(35, F + 0.5, -90)
                .driveReverseTangent(34, 50).driveReverseTangent(30, 90).driveLinear(12, 108, 0).park(B, 108, 0)));
    }

    @Test
    public void theEndPoseOfARouteIsCheckedToo() {
        // the route ends on a DRIVE whose last pose is 0.7 in through the left wall (the back edge toward it)
        List<Issue> l = check(AutoRoute.builder("x").start(35, F + 0.5, -90)
                .driveReverseTangent(34, 50).driveReverseTangent(30, 90).driveConstant(B - 0.7, 108, 0));
        assertEquals(1, wallErrors(l));
        assertEquals(3, wallError(l).stepIndex);
        assertTrue(wallError(l).message, wallError(l).message.contains("0.7 in"));
    }

    @Test
    public void everyFindingNamesTheWallRuleAndHowFarThroughTheRobotWent() {
        List<Issue> l = check(AutoRoute.builder("x").start(58, F, -90).driveConstant(30, F - 0.9, -90));
        Issue i = wallError(l);
        assertEquals(Severity.ERROR, i.severity);
        assertTrue(i.message, i.message.contains("leaves the field by up to 0.9 in"));
        assertTrue(i.message, i.message.contains("cannot drive or turn through a wall"));
    }

    // ---- 6. every wall and corner, facing every way: the real, off-center rectangle ------------------------------------

    /**
     * The corners of the robot at a pose, worked out here with plain trigonometry and the four distances (not with
     * Footprint, so the test does not share its mistakes): front-left, back-left, back-right, front-right.
     */
    private static double[][] corners(double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double c = Math.cos(h);
        double s = Math.sin(h);
        double[][] local = {{F, S}, {-B, S}, {-B, -S}, {F, -S}};
        double[][] out = new double[4][2];
        for (int i = 0; i < 4; i++) {
            out[i][0] = x + local[i][0] * c - local[i][1] * s;
            out[i][1] = y + local[i][0] * s + local[i][1] * c;
        }
        return out;
    }

    private static double[] minMax(double x, double y, double headingDeg) {
        double[] r = {Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (double[] p : corners(x, y, headingDeg)) {
            r[0] = Math.min(r[0], p[0]);
            r[1] = Math.max(r[1], p[0]);
            r[2] = Math.min(r[2], p[1]);
            r[3] = Math.max(r[3], p[1]);
        }
        return r;
    }

    /**
     * Situation: the robot placed touching the red wall (x = 0), the audience wall (y = 0), the rear wall (y = 144), and in
     *     the audience-left and rear-left corners, facing 0, 90, 180 and -90 degrees each: 20 starts. The distances from the
     *     position to the wall come from the four extents (front 9.77, back 8.03, sides 8.90), and the test checks them
     *     against the corners worked out by hand.
     * Expected: every touching start is legal (no finding at all); the same start 0.3 in closer to the wall (or both walls) is
     *     a G304 ERROR that says it sticks out of the field by 0.3 in; and a start 0.2 in away from the wall is legal too (the
     *     0.75 in contact tolerance).
     * Why: the robot is not centered on its position, so each of the four facings puts a different edge against the wall. A
     *     check that used half the length or half the width everywhere would pass one facing and fail another.
     */
    @Test
    public void everyFacingAtEveryWallAndCornerTouchesAndOneBitCloserClips() {
        // {x of the wall the position must keep from (or -1), y ...}: which walls the placement touches
        // each placement: touchesX = 0 (red wall) or none; touchesY = audience (y = 0), rear (y = 144) or none; the other
        // coordinate is a free place along the wall that keeps clear of FLOWER plates and the LOADING ZONE
        double[][] placements = {
                // wallX (0 = red wall, NaN none), wallY (0 = audience wall, 144 = rear wall, NaN none), freeX, freeY
                {0.0, Double.NaN, Double.NaN, 30.0},
                {Double.NaN, 0.0, 30.0, Double.NaN},
                {Double.NaN, 144.0, 30.0, Double.NaN},
                {0.0, 0.0, Double.NaN, Double.NaN},
                {0.0, 144.0, Double.NaN, Double.NaN}};
        int starts = 0;
        for (double[] p : placements) {
            for (int deg : new int[] {0, 90, 180, -90}) {
                double[] reach = {0, 0, 0, 0};
                // how far the footprint reaches from the position toward each wall: -x, +x, -y, +y (a corner can lead)
                double[] box = minMax(0.0, 0.0, deg);
                reach[0] = -box[0];
                reach[1] = box[1];
                reach[2] = -box[2];
                reach[3] = box[3];
                double x = Double.isNaN(p[0]) ? p[2] : reach[0];
                double y = Double.isNaN(p[1]) ? p[3] : (p[1] == 0.0 ? reach[2] : SIZE - reach[3]);
                // the footprint, worked out independently, touches the walls it should
                double[] mm = minMax(x, y, deg);
                if (!Double.isNaN(p[0])) {
                    assertEquals("red wall touched at " + deg, 0.0, mm[0], 1e-9);
                }
                if (!Double.isNaN(p[1])) {
                    assertEquals("audience/rear wall touched at " + deg, p[1], p[1] == 0.0 ? mm[2] : mm[3], 1e-9);
                }
                String where = "wall " + java.util.Arrays.toString(p) + " facing " + deg;
                assertTrue(where + ": " + check(AutoRoute.builder("x").start(x, y, deg).waitSeconds(1)),
                        check(AutoRoute.builder("x").start(x, y, deg).waitSeconds(1)).isEmpty());
                // 0.3 in closer to every wall it touches: a clip
                double cx = Double.isNaN(p[0]) ? x : x - 0.3;
                double cy = Double.isNaN(p[1]) ? y : (p[1] == 0.0 ? y - 0.3 : y + 0.3);
                List<Issue> clip = check(AutoRoute.builder("x").start(cx, cy, deg).waitSeconds(1));
                assertEquals(where + ": " + clip, 1, clip.size());
                assertEquals(where, Rule.START_G304, clip.get(0).rule);
                assertTrue(where + ": " + clip.get(0).message, clip.get(0).message.contains("out of the field by 0.3 in"));
                // 0.2 in away from the wall (a hand-placed robot): still a legal start
                double ax = Double.isNaN(p[0]) ? x : x + 0.2;
                double ay = Double.isNaN(p[1]) ? y : (p[1] == 0.0 ? y + 0.2 : y - 0.2);
                assertTrue(where + " 0.2 in off the wall: " + check(AutoRoute.builder("x").start(ax, ay, deg).waitSeconds(1)),
                        check(AutoRoute.builder("x").start(ax, ay, deg).waitSeconds(1)).isEmpty());
                starts++;
            }
        }
        assertEquals(20, starts);
    }

    /**
     * Situation: a start 0.3 in too close to the audience wall, with the front edge to it (facing -90), the back edge to it
     *     (facing +90) and a side edge to it (facing 0 and 180).
     * Expected: the finding says the robot's edge that faces the wall reaches 9.8, 8.0, 8.9 and 8.9 in from the center of
     *     rotation, which are the four extents of {@link BioBuzzConstants.Robot}.
     * Why: the message is what tells the team where to place the real robot; it must follow the extents, not a fixed number.
     */
    @Test
    public void theStartFindingSaysHowFarTheCenterOfRotationMustBeFromTheWallForEachFacing() {
        double[][] cases = {{-90, F}, {90, B}, {0, S}, {180, S}};
        for (double[] c : cases) {
            List<Issue> l = check(AutoRoute.builder("x").start(35, c[1] - 0.3, c[0]).waitSeconds(1));
            String m = l.get(0).message;
            assertTrue("facing " + c[0] + ": " + m, m.contains("reaches " + inches(c[1]) + " from the center"));
            assertTrue("facing " + c[0] + ": " + m, m.contains("wall at y = 0.0"));
        }
    }
}
