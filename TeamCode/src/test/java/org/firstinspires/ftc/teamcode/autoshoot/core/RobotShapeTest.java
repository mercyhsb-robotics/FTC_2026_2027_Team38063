package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link RobotShape}: where the robot's edges lie around its center of rotation. The numbers to remember: the front edge is
 * 9.77 in from the center of rotation, the back edge 8.03 in, each side edge 8.81 in (together 17.80 x 17.62 in). The tests take the numbers from BioBuzzConstants, so a new measurement does not break them.
 *
 * <p>Words used: <b>center of rotation</b> is the point the robot spins about, the position Pedro reports; <b>reach toward a
 * direction</b> is how far the outline extends from that point in a direction on the field while the robot faces some way.
 */
public class RobotShapeTest {
    private static final double EPS = 1e-9;

    /**
     * Situation: the shape built from the constants.
     * Expected: it carries the four extents, which add up to the overall length and width.
     */
    @Test
    public void theRealShapeIsTheFourExtentsOfTheConstants() {
        RobotShape s = RobotShape.fromConstants();
        assertEquals(BioBuzzConstants.Robot.FRONT_EXTENT_IN, s.frontIn(), EPS);
        assertEquals(BioBuzzConstants.Robot.REAR_EXTENT_IN, s.rearIn(), EPS);
        assertEquals(BioBuzzConstants.Robot.LEFT_EXTENT_IN, s.leftIn(), EPS);
        assertEquals(BioBuzzConstants.Robot.RIGHT_EXTENT_IN, s.rightIn(), EPS);
        assertEquals(BioBuzzConstants.Robot.LENGTH_IN, s.lengthIn(), 0.005);
        assertEquals(BioBuzzConstants.Robot.WIDTH_IN, s.widthIn(), 0.005);
        // the robot is NOT centered on its center of rotation: that is the whole point of the class
        assertTrue(s.frontIn() > s.rearIn() + 1.0);
    }

    @Test
    public void aCenteredShapeSplitsTheLengthAndWidthInHalf() {
        RobotShape c = RobotShape.centered(17.8, 17.8);
        assertEquals(8.9, c.frontIn(), EPS);
        assertEquals(8.9, c.rearIn(), EPS);
        assertEquals(8.9, c.leftIn(), EPS);
        assertEquals(8.9, c.rightIn(), EPS);
        assertEquals(Math.hypot(8.9, 8.9), c.reachIn(), EPS);
        assertEquals(8.9, c.smallestExtentIn(), EPS);
    }

    @Test
    public void everyDistanceMustBePositive() {
        double[][] bad = {{0, 8, 8, 8}, {9, -1, 8, 8}, {9, 8, 0, 8}, {9, 8, 8, Double.NaN}, {9, 8, 8, Double.POSITIVE_INFINITY}};
        for (double[] b : bad) {
            try {
                new RobotShape(b[0], b[1], b[2], b[3]);
                fail("accepted " + java.util.Arrays.toString(b));
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains("positive"));
            }
        }
    }

    /**
     * Situation: the circumscribed radius and the smallest extent of the real robot.
     * Expected: the farthest corner is a FRONT corner, hypot(front, side) = 13.16 in (not the 12.52 in of a centered rectangle); the
     *     nearest edge is the back, 8.03 in.
     */
    @Test
    public void theReachIsTheFarthestCornerAndTheSmallestExtentIsTheBack() {
        RobotShape s = RobotShape.fromConstants();
        double front = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
        double rear = BioBuzzConstants.Robot.REAR_EXTENT_IN;
        double side = Math.max(BioBuzzConstants.Robot.LEFT_EXTENT_IN, BioBuzzConstants.Robot.RIGHT_EXTENT_IN);
        assertTrue("the front is the long end", front > rear);
        assertEquals(Math.hypot(front, side), s.reachIn(), EPS);
        assertTrue(s.reachIn() > Math.hypot(BioBuzzConstants.Robot.LENGTH_IN / 2.0, BioBuzzConstants.Robot.WIDTH_IN / 2.0) + 0.5);
        assertEquals(rear, s.smallestExtentIn(), EPS);
        // an independent brute force over the four corners
        double far = 0.0;
        for (double f : new double[] {s.frontIn(), -s.rearIn()}) {
            for (double l : new double[] {s.leftIn(), -s.rightIn()}) {
                far = Math.max(far, Math.hypot(f, l));
            }
        }
        assertEquals(far, s.reachIn(), EPS);
    }

    /**
     * Situation: the reach toward each wall with the robot facing the four axis directions.
     * Expected: facing the wall front-first it is the front extent; facing away, the rear extent; sideways, a side extent.
     */
    @Test
    public void theReachTowardAWallIsTheEdgeThatFacesIt() {
        RobotShape s = RobotShape.fromConstants();
        double toAudienceWall = Math.toRadians(-90);     // the audience wall is at y = 0, in the direction -90 degrees
        double front = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
        double rear = BioBuzzConstants.Robot.REAR_EXTENT_IN;
        double left = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
        double right = BioBuzzConstants.Robot.RIGHT_EXTENT_IN;
        // facing 0 degrees (toward +x) with the wall at -y: the robot's RIGHT side is toward the wall; facing 180 the LEFT side is
        assertEquals("facing the wall: the front", front, s.reachTowardIn(Math.toRadians(-90), toAudienceWall), EPS);
        assertEquals("facing away from it: the back", rear, s.reachTowardIn(Math.toRadians(90), toAudienceWall), EPS);
        assertEquals("sideways: a side", right, s.reachTowardIn(Math.toRadians(0), toAudienceWall), EPS);
        assertEquals("sideways the other way: the other side", left, s.reachTowardIn(Math.toRadians(180), toAudienceWall), EPS);
        // the same four facings against the red wall (x = 0, direction 180 degrees)
        double toRedWall = Math.PI;
        assertEquals(front, s.reachTowardIn(Math.PI, toRedWall), EPS);
        assertEquals(rear, s.reachTowardIn(0.0, toRedWall), EPS);
        assertEquals(left, s.reachTowardIn(Math.PI / 2.0, toRedWall), EPS);
        assertEquals(right, s.reachTowardIn(-Math.PI / 2.0, toRedWall), EPS);
    }

    /**
     * Situation: 20000 random headings and directions.
     * Expected: the reach toward a direction equals the largest projection of the four corners onto that direction, worked out here
     *     with plain trigonometry (no use of the class under test).
     */
    @Test
    public void theReachTowardAnyDirectionIsTheFarthestCornerAlongIt() {
        RobotShape s = RobotShape.fromConstants();
        Random rnd = new Random(5);
        for (int k = 0; k < 20000; k++) {
            double heading = (rnd.nextDouble() - 0.5) * 2.0 * Math.PI;
            double dir = (rnd.nextDouble() - 0.5) * 2.0 * Math.PI;
            double best = Double.NEGATIVE_INFINITY;
            for (double f : new double[] {s.frontIn(), -s.rearIn()}) {
                for (double l : new double[] {s.leftIn(), -s.rightIn()}) {
                    double x = f * Math.cos(heading) - l * Math.sin(heading);
                    double y = f * Math.sin(heading) + l * Math.cos(heading);
                    best = Math.max(best, x * Math.cos(dir) + y * Math.sin(dir));
                }
            }
            assertEquals("heading " + heading + " direction " + dir, best, s.reachTowardIn(heading, dir), 1e-9);
        }
    }

    @Test
    public void equalShapesAreEqualAndDifferentOnesAreNot() {
        RobotShape same = new RobotShape(BioBuzzConstants.Robot.FRONT_EXTENT_IN, BioBuzzConstants.Robot.REAR_EXTENT_IN,
                BioBuzzConstants.Robot.LEFT_EXTENT_IN, BioBuzzConstants.Robot.RIGHT_EXTENT_IN);
        assertEquals(RobotShape.fromConstants(), same);
        assertEquals(RobotShape.fromConstants().hashCode(), same.hashCode());
        assertFalse(RobotShape.fromConstants().equals(RobotShape.centered(BioBuzzConstants.Robot.LENGTH_IN, BioBuzzConstants.Robot.WIDTH_IN)));
        assertTrue(RobotShape.fromConstants().toString(), RobotShape.fromConstants().toString().contains(String.format(java.util.Locale.ROOT, "front %.2f in", BioBuzzConstants.Robot.FRONT_EXTENT_IN)));
    }
}
