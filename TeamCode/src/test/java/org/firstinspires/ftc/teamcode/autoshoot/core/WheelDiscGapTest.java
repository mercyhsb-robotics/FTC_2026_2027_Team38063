package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Arrays;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link Footprint#wheelDiscGap}: the signed gap between a flat intake wheel, as the ROUND disc it is, and a rectangle; and the two real wheels of
 * {@link IntakeWheel}, which AUTO is not one of.
 *
 * <p>Why a round disc: the FLOWER wiggle turns the robot by up to 20 degrees with a wheel close to a line it must not cross. The square that
 * {@link Footprint#wheelDisc} puts around a wheel has corners that stick out about 0.3 in beyond the real disc when it is turned, and would report a
 * collision that is not there. The gap here is a TRUE distance, so it can be trusted by the route sweep ({@code Sweep}), which relies on a gap never
 * changing by more than the wheel moves.
 *
 * <p>How it works: the answer is compared with a brute-force measurement, the distance from the rectangle to a thousand points around the rim of the disc.
 */
public class WheelDiscGapTest {
    private static final double RADIUS = BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0;
    private static final Rect BOX = new Rect(10.0, 16.0, 20.0, 30.0);

    /** The distance from the disc (center and radius) to the box found the slow way: the nearest rim point, or minus the depth if the middle is inside. */
    private static double bruteForce(Pose2 pose, IntakeWheel w) {
        Point2 c = Footprint.wheelCenter(pose, w);
        double nearest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 4000; i++) {
            double a = Math.PI * 2.0 * i / 4000.0;
            double x = c.x + RADIUS * Math.cos(a);
            double y = c.y + RADIUS * Math.sin(a);
            double dx = Math.max(Math.max(BOX.minX - x, 0.0), x - BOX.maxX);
            double dy = Math.max(Math.max(BOX.minY - y, 0.0), y - BOX.maxY);
            nearest = Math.min(nearest, Math.hypot(dx, dy));
        }
        double centerDx = Math.max(Math.max(BOX.minX - c.x, 0.0), c.x - BOX.maxX);
        double centerDy = Math.max(Math.max(BOX.minY - c.y, 0.0), c.y - BOX.maxY);
        if (Math.hypot(centerDx, centerDy) > RADIUS) {
            return nearest;                                  // clear of the box: the nearest rim point is the answer
        }
        return Double.NaN;                                   // touching or inside: the formula is checked separately
    }

    @Test
    public void theGapIsTheTrueDistanceToTheNearestPointOfTheDisc() {
        int compared = 0;
        for (IntakeWheel w : IntakeWheel.physical()) {
            for (double x = 0.0; x <= 30.0; x += 2.5) {
                for (double y = 8.0; y <= 42.0; y += 2.5) {
                    for (int deg = -180; deg < 180; deg += 30) {
                        Pose2 p = Pose2.ofDegrees(x, y, deg);
                        double brute = bruteForce(p, w);
                        if (!Double.isNaN(brute)) {
                            assertEquals(w + " " + p, brute, Footprint.wheelDiscGap(p, w, BOX), 1e-4);
                            compared++;
                        }
                    }
                }
            }
        }
        assertTrue("enough clear poses were compared: " + compared, compared > 500);
    }

    @Test
    public void aDiscThatJustTouchesTheBoxHasGapZeroAndOneThatOverlapsHasNegativeGap() {
        // facing +x with the pose point at (x, 25): the wheel's middle is 8.37 in ahead and 6.1 in to the left, so at y = 31.1 - the middle is level with the box's top... use the left wheel
        // placed so that its disc's near edge touches the box's left side x = 10: middle x = 10 - 1.4
        double forward = BioBuzzConstants.Robot.FRONT_EXTENT_IN - RADIUS;
        double side = BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN;
        Pose2 touching = new Pose2(10.0 - RADIUS - forward, 25.0 - side, 0.0);          // wheel middle at (8.6, 25.0), box starts at x = 10
        assertEquals(0.0, Footprint.wheelDiscGap(touching, IntakeWheel.LEFT, BOX), 1e-9);
        Pose2 inside = new Pose2(touching.x + 0.3, touching.y, 0.0);
        assertEquals("0.3 in into the box", -0.3, Footprint.wheelDiscGap(inside, IntakeWheel.LEFT, BOX), 1e-9);
        Pose2 apart = new Pose2(touching.x - 0.7, touching.y, 0.0);
        assertEquals("0.7 in away", 0.7, Footprint.wheelDiscGap(apart, IntakeWheel.LEFT, BOX), 1e-9);
        // with the middle of the wheel inside the box the disc has to slide out through the nearest side: middle 1 in from the left side, radius 1.4
        Pose2 deep = new Pose2(11.0 - forward, 25.0 - side, 0.0);
        assertEquals(-(1.0 + RADIUS), Footprint.wheelDiscGap(deep, IntakeWheel.LEFT, BOX), 1e-9);
    }

    @Test
    public void aRoundDiscIsClearWhereASquareAroundItIsNot() {
        // turned 45 degrees the square around the wheel has a corner sticking out 0.58 in past the disc: that corner pokes into a box the real disc misses
        double forward = BioBuzzConstants.Robot.FRONT_EXTENT_IN - RADIUS;
        double side = BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN;
        Pose2 pose = Pose2.ofDegrees(0.0, 0.0, 45.0);
        Point2 c = Footprint.wheelCenter(pose, IntakeWheel.LEFT);
        // a box whose left side is 0.2 in clear of the disc's own rim
        Rect box = new Rect(c.x + RADIUS + 0.2, c.x + RADIUS + 6.0, c.y - 3.0, c.y + 3.0);
        assertEquals(0.2, Footprint.wheelDiscGap(pose, IntakeWheel.LEFT, box), 1e-9);
        assertTrue("the square corner is deeper in than the disc", Footprint.wheelDisc(pose, IntakeWheel.LEFT).signedGap(box) < 0.2 - 0.1);
        assertTrue(forward > 0 && side > 0);
    }

    @Test
    public void movingTheWheelChangesTheGapByNoMoreThanTheWheelMoves() {
        // the property the sweep relies on: a gap is 1-Lipschitz
        Pose2 a = Pose2.ofDegrees(3.0, 22.0, 0.0);
        double last = Footprint.wheelDiscGap(a, IntakeWheel.RIGHT, BOX);
        for (double dx = 0.05; dx <= 6.0; dx += 0.05) {
            double now = Footprint.wheelDiscGap(new Pose2(a.x + dx, a.y, 0.0), IntakeWheel.RIGHT, BOX);
            assertTrue(now - last <= 0.05 + 1e-9 && last - now <= 0.05 + 1e-9);
            last = now;
        }
        for (int deg = 0; deg < 90; deg++) {
            Pose2 p0 = Pose2.ofDegrees(8.0, 20.0, deg);
            Pose2 p1 = Pose2.ofDegrees(8.0, 20.0, deg + 1);
            double moved = Footprint.wheelCenter(p0, IntakeWheel.LEFT).distanceTo(Footprint.wheelCenter(p1, IntakeWheel.LEFT));
            assertTrue(Math.abs(Footprint.wheelDiscGap(p0, IntakeWheel.LEFT, BOX) - Footprint.wheelDiscGap(p1, IntakeWheel.LEFT, BOX)) <= moved + 1e-9);
        }
    }

    // ---- the enum ----------------------------------------------------------------------------------------------------------

    @Test
    public void autoIsARequestNotAWheel() {
        assertTrue(IntakeWheel.AUTO.isAuto());
        assertFalse(IntakeWheel.LEFT.isAuto() || IntakeWheel.RIGHT.isAuto());
        try {
            IntakeWheel.AUTO.sign();
            fail("AUTO has no side");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("not a wheel"));
        }
        try {
            IntakeWheel.AUTO.other();
            fail("AUTO has no other wheel");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("not a wheel"));
        }
        // the real wheels are LEFT then RIGHT, mirror images, and a fresh array each time
        IntakeWheel[] real = IntakeWheel.physical();
        assertEquals(Arrays.asList(IntakeWheel.LEFT, IntakeWheel.RIGHT), Arrays.asList(real));
        real[0] = IntakeWheel.AUTO;
        assertEquals("changing the returned array does not change the enum", IntakeWheel.LEFT, IntakeWheel.physical()[0]);
        assertEquals(1.0, IntakeWheel.LEFT.sign(), 0.0);
        assertEquals(-1.0, IntakeWheel.RIGHT.sign(), 0.0);
        assertEquals(IntakeWheel.RIGHT, IntakeWheel.LEFT.other());
        assertEquals(IntakeWheel.LEFT, IntakeWheel.RIGHT.other());
        // AUTO is the default value of a request: asking for the footprint of a wheel it does not name is refused
        try {
            Footprint.wheelCenter(Pose2.ofDegrees(0, 0, 0), IntakeWheel.AUTO);
            fail("a wheel position needs a real wheel");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("AUTO"));
        }
    }
}
