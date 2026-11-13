package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Before;
import org.junit.Test;

/**
 * Tests for {@link PoseCorrector}: the camera should pull a drifted odometry position back toward the truth,
 * but only when the sighting is trustworthy.
 *
 * <p>The "camera" here is a small virtual one: it looks at the HIVE tags from the robot's TRUE position and
 * hands the result to the same {@link ClusterAnalyzer} the TeleOp uses. The odometry position given to the
 * corrector is the true position plus a made-up drift.
 */
public class PoseCorrectorTest {
    private final FieldModel field = new FieldModel();
    private final CameraModel camera = CameraModel.fromConstants();
    private final ClusterAnalyzer analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());

    /** Where the robot really is: in front of the red audience-side cell, aiming roughly at it. */
    private final Pose2 truth = Pose2.ofDegrees(55.0, 14.0, -97.0);
    private final CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);

    private PoseCorrector.Config config;

    @Before
    public void setUp() {
        config = PoseCorrector.Config.fromConstants();
        config.enabled = true;          // the shipped default is "off"; these tests are about the "on" behaviour
    }

    // ------------------------------------------------------------------ helpers

    /** What the camera reports for the chosen tags of the cell when the robot is really at {@code truth}. */
    private List<TagObservation> observe(boolean cellIsUp, int[] tagIndexes) {
        List<TagObservation> out = new ArrayList<TagObservation>();
        for (int i : tagIndexes) {
            double x = cell.tagFieldX(i);
            double y = cellIsUp ? cell.tagRowY : field.center() + cell.side.ySign() * 11.65;
            double z = cellIsUp ? cell.tagRowZ : cell.downTagRowZ;
            double f = truth.forwardOf(x, y);
            double l = truth.leftOf(x, y);
            double[] tt = camera.txTyOf(f, l, z);
            out.add(TagObservation.withPosition(cell.baseTagId + i, tt[0], tt[1], 0.5,
                    camera.azimuthOf(tt[0], tt[1]), f, l, z));
        }
        return out;
    }

    private static final int[] ALL_FOUR = {0, 1, 2, 3};

    private List<ClusterView> viewsFor(Pose2 odometry, boolean cellIsUp, int[] tagIndexes) {
        return analyzer.analyze(Alliance.RED, odometry.heading, observe(cellIsUp, tagIndexes));
    }

    private static VisionFrame freshFrame(double now) {
        return new VisionFrame(VisionFrame.Status.OK, true, 0.05, now, new ArrayList<TagObservation>());
    }

    /** The odometry position: the truth plus a drift of (dx, dy) inches, same heading. */
    private Pose2 drifted(double dx, double dy) {
        return new Pose2(truth.x + dx, truth.y + dy, truth.heading);
    }

    private double errorOf(Pose2 odometry) {
        return Math.hypot(odometry.x - truth.x, odometry.y - truth.y);
    }

    // ------------------------------------------------------------------ the tests

    @Test
    public void doesNothingWhileSwitchedOff() {
        config.enabled = false;
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(3, -2);
        assertNull(corrector.update(1.0, odo, 0, 0, freshFrame(1.0), viewsFor(odo, true, ALL_FOUR)));
        assertEquals(0, corrector.appliedCount());
    }

    @Test
    public void repeatedSightingsPullADriftedPositionBackToTheTruth() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(4.0, -3.0);                          // 5 in of drift, inside the 10 in gate
        double previousError = errorOf(odo);
        double now = 1.0;
        for (int i = 0; i < 25; i++) {
            PoseCorrector.Correction fix = corrector.update(now, odo, 0, 0, freshFrame(now),
                    viewsFor(odo, true, ALL_FOUR));
            if (fix != null) {
                odo = new Pose2(odo.x + fix.dx, odo.y + fix.dy, odo.heading);
                double error = errorOf(odo);
                assertTrue("each correction must reduce the error: " + previousError + " -> " + error,
                        error < previousError);
                previousError = error;
            }
            now += config.minIntervalSec;
        }
        assertTrue("error should be nearly gone, was " + previousError, previousError < 0.1);
        assertEquals("the heading is never touched", truth.heading, odo.heading, 1e-12);
    }

    @Test
    public void oneCorrectionIsAFractionOfTheDisagreementAndNeverLargerThanTheStepLimit() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(2.0, 0.0);                           // 2 in: gain decides
        PoseCorrector.Correction small = corrector.update(1.0, odo, 0, 0, freshFrame(1.0), viewsFor(odo, true, ALL_FOUR));
        assertNotNull(small);
        assertEquals(config.gain * 2.0, Math.hypot(small.dx, small.dy), 1e-6);
        assertTrue("the step points back toward the truth", small.dx < 0);

        PoseCorrector second = new PoseCorrector(config);
        Pose2 far = drifted(9.0, 0.0);                           // 9 in: the step limit decides
        PoseCorrector.Correction big = second.update(1.0, far, 0, 0, freshFrame(1.0), viewsFor(far, true, ALL_FOUR));
        assertNotNull(big);
        assertEquals(config.maxStepIn, Math.hypot(big.dx, big.dy), 1e-6);
        assertEquals(9.0, big.disagreementIn, 1e-6);
    }

    @Test
    public void aSightingThatDisagreesByMoreThanTheGateIsThrownAway() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(config.gateIn + 5.0, 0.0);
        assertNull(corrector.update(1.0, odo, 0, 0, freshFrame(1.0), viewsFor(odo, true, ALL_FOUR)));
        assertEquals(0, corrector.appliedCount());
        assertEquals(1, corrector.rejectedCount());
        assertTrue(corrector.status(), corrector.status().contains("ignored"));
    }

    @Test
    public void doesNotCorrectWhileTheRobotIsMovingOrTurningFast() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(2.0, 0.0);
        List<ClusterView> views = viewsFor(odo, true, ALL_FOUR);
        assertNull("too fast", corrector.update(1.0, odo, config.maxSpeedInS + 1.0, 0, freshFrame(1.0), views));
        assertNull("turning too fast", corrector.update(2.0, odo, 0,
                Math.toRadians(config.maxOmegaDegS + 10.0), freshFrame(2.0), views));
        assertNotNull("standing still is fine", corrector.update(3.0, odo, 0, 0, freshFrame(3.0), views));
    }

    @Test
    public void doesNotCorrectFromAnOldOrRepeatedImage() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(2.0, 0.0);
        List<ClusterView> views = viewsFor(odo, true, ALL_FOUR);
        VisionFrame old = new VisionFrame(VisionFrame.Status.OK, true, config.maxFrameAgeSec + 0.1, 1.0,
                new ArrayList<TagObservation>());
        VisionFrame repeated = new VisionFrame(VisionFrame.Status.OK, false, 0.05, 1.0,
                new ArrayList<TagObservation>());
        VisionFrame notConnected = VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, 1.0);
        assertNull(corrector.update(1.0, odo, 0, 0, old, views));
        assertNull(corrector.update(1.0, odo, 0, 0, repeated, views));
        assertNull(corrector.update(1.0, odo, 0, 0, notConnected, views));
        assertEquals(0, corrector.appliedCount());
    }

    @Test
    public void waitsBetweenCorrections() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(2.0, 0.0);
        List<ClusterView> views = viewsFor(odo, true, ALL_FOUR);
        assertNotNull(corrector.update(1.0, odo, 0, 0, freshFrame(1.0), views));
        assertNull("too soon", corrector.update(1.0 + config.minIntervalSec / 2, odo, 0, 0,
                freshFrame(1.0 + config.minIntervalSec / 2), views));
        assertNotNull("after the wait", corrector.update(1.0 + config.minIntervalSec + 0.01, odo, 0, 0,
                freshFrame(1.0 + config.minIntervalSec + 0.01), views));
    }

    @Test
    public void needsEnoughTagsAndAnUpCell() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(2.0, 0.0);
        assertNull("one tag is not enough", corrector.update(1.0, odo, 0, 0, freshFrame(1.0),
                viewsFor(odo, true, new int[]{1})));
        assertNull("a cell that is tipped DOWN says nothing reliable about the robot position",
                corrector.update(2.0, odo, 0, 0, freshFrame(2.0), viewsFor(odo, false, ALL_FOUR)));
        assertNotNull("two tags are enough", corrector.update(3.0, odo, 0, 0, freshFrame(3.0),
                viewsFor(odo, true, new int[]{0, 3})));
    }

    @Test
    public void whenNoTagIsInViewNothingHappens() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(2.0, 0.0);
        assertNull(corrector.update(1.0, odo, 0, 0, freshFrame(1.0), new ArrayList<ClusterView>()));
        assertTrue(corrector.status(), corrector.status().contains("no usable cluster"));
    }
}
