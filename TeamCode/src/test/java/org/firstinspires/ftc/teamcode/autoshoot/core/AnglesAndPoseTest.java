package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class AnglesAndPoseTest {
    private static final double EPS = 1e-9;

    @Test
    public void normalizeMapsIntoSignedRangeWithPlusPiIncluded() {
        assertEquals(Math.PI, Angles.normalize(Math.PI), EPS);
        assertEquals(Math.PI, Angles.normalize(-Math.PI), EPS);
        assertEquals(Math.toRadians(-90), Angles.normalize(Math.toRadians(270)), EPS);
        assertEquals(Math.toRadians(10), Angles.normalize(Math.toRadians(370)), EPS);
        assertEquals(0.0, Angles.normalize(4 * Math.PI), EPS);
    }

    @Test
    public void errorIsShortestSignedTurnAcrossTheSeam() {
        assertEquals(Math.toRadians(2), Angles.error(Math.toRadians(179), Math.toRadians(-179)), EPS);
        assertEquals(Math.toRadians(-2), Angles.error(Math.toRadians(-179), Math.toRadians(179)), EPS);
        assertEquals(Math.toRadians(20), Angles.error(Math.toRadians(350), Math.toRadians(10)), EPS);
        assertEquals(Math.toRadians(-20), Angles.error(Math.toRadians(10), Math.toRadians(350)), EPS);
        assertEquals(0.0, Angles.error(1.0, 1.0), EPS);
    }

    @Test
    public void oppositeFlipsHalfATurn() {
        assertEquals(Math.toRadians(90), Angles.opposite(Math.toRadians(-90)), EPS);
        assertEquals(Math.toRadians(-90), Angles.opposite(Math.toRadians(90)), EPS);
    }

    @Test
    public void poseNormalizesItsHeading() {
        Pose2 p = Pose2.ofDegrees(35, 9.2, 270);
        assertEquals(-90.0, p.headingDegrees(), 1e-9);
        assertTrue(p.heading > -Math.PI && p.heading <= Math.PI);
    }

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

    @Test
    public void rearShooterAtNegativeForwardEndsUpOnTheHiveSideForTheTeamStartPose() {
        // Team start pose: (35, 9.2) facing -90 deg (intake toward the audience wall). A point 8.9 in
        // behind the center must be toward +y, i.e. toward the HIVE.
        Pose2 start = Pose2.ofDegrees(35, 9.2, -90);
        assertEquals(35.0, start.fieldX(-8.9, 0), 1e-9);
        assertEquals(18.1, start.fieldY(-8.9, 0), 1e-9);
    }
}
