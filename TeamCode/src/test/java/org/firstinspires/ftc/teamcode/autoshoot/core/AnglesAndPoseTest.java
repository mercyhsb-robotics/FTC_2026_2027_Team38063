package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Checks the two smallest building blocks of the auto-shoot code: {@link Angles} (wrapping angles and finding the
 * shortest turn) and {@link Pose2} (where the robot is, which way it faces, and how to convert a point between the
 * robot's own frame and the field frame).
 *
 * <p>How it works: these are 'unit tests', the smallest kind of test. Each test calls one method with numbers we worked
 * out by hand and uses JUnit's {@code assertEquals} or {@code assertTrue} to check the answer. No robot, hardware or
 * clock is needed, so the whole file runs in milliseconds on a PC.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*AnglesAndPoseTest"} to run only
 * this class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Radians:</b> the angle unit the robot code uses. A half turn is pi (about 3.14159) and a full turn is 2
 *       pi. The tests write degrees and convert with {@code Math.toRadians}.</li>
 *   <li><b>Heading:</b> the direction the robot's front faces on the field. 0 degrees is field +x and a positive
 *       turn is counter-clockwise.</li>
 *   <li><b>Seam:</b> the spot where an angle jumps from +180 to -180 degrees. Angle code must not break there.</li>
 *   <li><b>Epsilon (EPS) / tolerance:</b> the 'close enough' gap allowed when comparing decimal numbers. Computers
 *       round tiny bits off, so {@code assertEquals(expected, actual, EPS)} passes when the two differ by no more than
 *       EPS.</li>
 * </ul>
 */
public class AnglesAndPoseTest {
    /**
     * Allowed rounding error when comparing two decimal numbers: one billionth. Very tight, because these tests use
     * exact maths and no measurements.
     */
    private static final double EPS = 1e-9;

    /**
     * Situation: angles outside the normal range (270 degrees, 370 degrees, 4 pi) plus +pi and -pi themselves.
     * Expected: each comes back inside (-pi, pi], and -pi is turned into +pi.
     * Why: all turn maths assumes one standard range, so every direction has exactly one name.
     */
    @Test
    public void normalizeMapsIntoSignedRangeWithPlusPiIncluded() {
        assertEquals(Math.PI, Angles.normalize(Math.PI), EPS);
        assertEquals(Math.PI, Angles.normalize(-Math.PI), EPS);
        assertEquals(Math.toRadians(-90), Angles.normalize(Math.toRadians(270)), EPS);
        assertEquals(Math.toRadians(10), Angles.normalize(Math.toRadians(370)), EPS);
        assertEquals(0.0, Angles.normalize(4 * Math.PI), EPS);
    }

    /**
     * Situation: pairs of headings on either side of the +/-180 degree seam (179 and -179) and of the 0/360 line (350
     *     and 10), plus two equal headings.
     * Expected: the error is the SHORT way round (+2, -2, +20, -20 degrees), never the long way (358 degrees), and
     *     equal headings give 0.
     * Why: when the robot aligns on the HIVE it must turn the nearest way.
     */
    @Test
    public void errorIsShortestSignedTurnAcrossTheSeam() {
        assertEquals(Math.toRadians(2), Angles.error(Math.toRadians(179), Math.toRadians(-179)), EPS);
        assertEquals(Math.toRadians(-2), Angles.error(Math.toRadians(-179), Math.toRadians(179)), EPS);
        assertEquals(Math.toRadians(20), Angles.error(Math.toRadians(350), Math.toRadians(10)), EPS);
        assertEquals(Math.toRadians(-20), Angles.error(Math.toRadians(10), Math.toRadians(350)), EPS);
        assertEquals(0.0, Angles.error(1.0, 1.0), EPS);
    }

    /**
     * Situation: headings of -90 and +90 degrees.
     * Expected: the opposite heading is half a turn away (+90 and -90).
     * Why: the shooter fires out the BACK, so 'point the shooter at the HIVE' means 'point the front the other way'.
     *     The red/blue field flip also adds half a turn.
     */
    @Test
    public void oppositeFlipsHalfATurn() {
        assertEquals(Math.toRadians(90), Angles.opposite(Math.toRadians(-90)), EPS);
        assertEquals(Math.toRadians(-90), Angles.opposite(Math.toRadians(90)), EPS);
    }

    /**
     * Situation: a pose built with a heading of 270 degrees.
     * Expected: the stored heading is -90 degrees and lies in (-pi, pi].
     * Why: poses that face the same way must hold the same number.
     */
    @Test
    public void poseNormalizesItsHeading() {
        Pose2 p = Pose2.ofDegrees(35, 9.2, 270);
        // 1e-9 degrees is effectively exact: only floating-point rounding is allowed.
        assertEquals(-90.0, p.headingDegrees(), 1e-9);
        assertTrue(p.heading > -Math.PI && p.heading <= Math.PI);
    }

    /**
     * Situation: a robot at field (10, 20) facing +90 degrees (toward field +y).
     * Expected: a point 5 in ahead of it is at field (10, 25), a point 5 in to its left is at field (5, 20), and
     *     converting a field point back to 'forward' and 'left' returns the numbers we started with.
     * Why: the shooter, camera and tags are all described as (forward, left), so a sign mistake would aim everything
     *     the wrong way. The 'left-hand rule' means +left is 90 degrees counter-clockwise from +forward.
     */
    @Test
    public void robotFrameTransformsFollowTheLeftHandRule() {
        // Heading +90 deg: forward is field +y, left is field -x.
        Pose2 p = Pose2.ofDegrees(10, 20, 90);
        assertEquals(10.0, p.fieldX(5, 0), EPS);
        assertEquals(25.0, p.fieldY(5, 0), EPS);
        assertEquals(5.0, p.fieldX(0, 5), EPS);
        assertEquals(20.0, p.fieldY(0, 5), EPS);
        // The round trip must be the identity.
        double fx = p.fieldX(-8.9, 3.0);
        double fy = p.fieldY(-8.9, 3.0);
        assertEquals(-8.9, p.forwardOf(fx, fy), EPS);
        assertEquals(3.0, p.leftOf(fx, fy), EPS);
    }

    /**
     * Situation: the robot at the B1 autonomous start pose, facing the audience wall, and a point 8.9 in behind its
     *     center (where the rear shooter's ball exit is).
     * Expected: that point is at field (35, 18.1), 8.9 in further from the audience wall than the robot center.
     * Why: the shooter fires out the back, which here is toward the HIVE.
     */
    @Test
    public void rearShooterAtNegativeForwardEndsUpOnTheHiveSideForTheTeamStartPose() {
        // The B1 autonomous start pose (RedAlliance_B1): (35, 9.2) facing -90 deg (intake toward the audience wall). A
        // point 8.9 in behind the center must be toward +y, i.e. toward the HIVE. (The TeleOp default start is now
        // (60, 8.9) in BioBuzzConstants.Controls; any pose would do for this check.)
        Pose2 start = Pose2.ofDegrees(35, 9.2, -90);
        // 1e-9 in: only floating-point rounding is allowed.
        assertEquals(35.0, start.fieldX(-8.9, 0), 1e-9);
        assertEquals(18.1, start.fieldY(-8.9, 0), 1e-9);
    }
}
