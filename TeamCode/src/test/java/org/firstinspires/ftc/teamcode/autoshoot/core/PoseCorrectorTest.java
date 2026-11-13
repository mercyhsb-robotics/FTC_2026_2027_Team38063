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
 * Tests for {@link PoseCorrector}: the camera should pull a drifted odometry position back toward the truth, but only
 * when the sighting is trustworthy.
 *
 * <p>Background: odometry (here the goBILDA Pinpoint) is very accurate over short distances but slowly drifts. The
 * HIVE's AprilTags sit at known places, so every sighting also tells us where the robot must be standing. PoseCorrector
 * nudges the odometry position toward that answer, cautiously. It is OFF by default (Vision.POSE_CORRECTION_ENABLED)
 * until the camera mount has been checked on the real robot.
 *
 * <p>How it works: the 'camera' here is a small virtual one: it looks at the HIVE tags from the robot's TRUE position
 * and hands the result to the same {@link ClusterAnalyzer} the TeleOp uses. The odometry position given to the
 * corrector is the true position plus a made-up drift. So the right answer is always known: the truth.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*PoseCorrectorTest"} to run only
 * this class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Drift:</b> the slow error that builds up in odometry (wheels slip, the heading creeps).</li>
 *   <li><b>Virtual camera:</b> test code that computes what a real camera WOULD report for a known robot position,
 *       so no camera is needed.</li>
 *   <li><b>Gate:</b> a limit beyond which a reading is thrown away as a mistake (here, a camera answer that
 *       disagrees with odometry by more than a set number of inches).</li>
 *   <li><b>Gain:</b> the fraction of the disagreement corrected in one step. A small gain gives a smooth, safe pull
 *       rather than a jump.</li>
 *   <li><b>Fixture:</b> the shared setup for the tests (the fields below and {@code setUp}). JUnit builds a fresh
 *       copy for every test.</li>
 *   <li><b>Tolerance:</b> how far two decimal numbers may differ and still count as equal, to allow for
 *       rounding.</li>
 * </ul>
 */
public class PoseCorrectorTest {
    /**
     * The field map, the camera as mounted (from BioBuzzConstants) and the real analyzer: the shared fixture (these
     * three fields).
     */
    private final FieldModel field = new FieldModel();
    private final CameraModel camera = CameraModel.fromConstants();
    private final ClusterAnalyzer analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());

    /**
     * Where the robot really is: in front of the red audience-side cell, with its back (the shooter and camera) aiming
     * roughly at it. The heading of -97 degrees is the front pointing at the audience wall, turned 7 degrees.
     */
    private final Pose2 truth = Pose2.ofDegrees(55.0, 14.0, -97.0);
    /** The cell the virtual camera looks at: the RED audience-side cell. */
    private final CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);

    /** The corrector's settings. Rebuilt from BioBuzzConstants before every test so tests cannot affect each other. */
    private PoseCorrector.Config config;

    /** Runs before each test: loads the real settings, then switches the feature ON. */
    @Before
    public void setUp() {
        config = PoseCorrector.Config.fromConstants();
        config.enabled = true;          // the shipped default is "off"; these tests are about the "on" behaviour
    }

    // ------------------------------------------------------------------ helpers

    /**
     * What the camera reports for the chosen tags of the cell when the robot is really at {@code truth}.
     *
     * <p>The virtual camera: for each chosen tag it works out where the tag is on the field, converts that into the
     * robot's frame (forward, left, height) and then into the tx/ty angles a real Limelight would report (degrees right
     * of and above its crosshair), and packs it all into a {@link TagObservation}. It simplifies: the numbers are
     * perfect (no noise), the tags are never out of the camera's view, and the tag area is a fixed 0.5.
     *
     * @param cellIsUp true for an UP cell; false makes the tags sit where a tipped DOWN cell's would
     * @param tagIndexes which of the cell's four tags (0 to 3) the camera can see
     */
    private List<TagObservation> observe(boolean cellIsUp, int[] tagIndexes) {
        List<TagObservation> out = new ArrayList<TagObservation>();
        for (int i : tagIndexes) {
            double x = cell.tagFieldX(i);
            // A DOWN cell's tag row is 11.65 in out from the pivot plane (y = 72) on its own side and lower
            // (downTagRowZ); an UP cell's row is at tagRowY and tagRowZ.
            double y = cellIsUp ? cell.tagRowY : field.center() + cell.side.ySign() * 11.65;
            double z = cellIsUp ? cell.tagRowZ : cell.downTagRowZ;
            double f = truth.forwardOf(x, y);
            double l = truth.leftOf(x, y);
            // tx and ty: the angles, in degrees, at which a real Limelight would see this tag.
            double[] tt = camera.txTyOf(f, l, z);
            out.add(TagObservation.withPosition(cell.baseTagId + i, tt[0], tt[1], 0.5,
                    camera.azimuthOf(tt[0], tt[1]), f, l, z));
        }
        return out;
    }

    /** The indexes of all four tags of a cell (the tags are numbered 0 to 3 in ID order). */
    private static final int[] ALL_FOUR = {0, 1, 2, 3};

    /**
     * Runs the real ClusterAnalyzer on what the virtual camera sees. The odometry position is passed in only for its
     * heading, which is how the TeleOp calls the analyzer.
     */
    private List<ClusterView> viewsFor(Pose2 odometry, boolean cellIsUp, int[] tagIndexes) {
        return analyzer.analyze(Alliance.RED, odometry.heading, observe(cellIsUp, tagIndexes));
    }

    /**
     * A camera snapshot saying: link healthy, a NEW image just arrived, it is 0.05 s old, read at time {@code now}. Its
     * tag list is empty because the tags reach the corrector separately as ClusterViews.
     */
    private static VisionFrame freshFrame(double now) {
        return new VisionFrame(VisionFrame.Status.OK, true, 0.05, now, new ArrayList<TagObservation>());
    }

    /**
     * The odometry position: the truth plus a drift of (dx, dy) inches, same heading. (Only position drifts here;
     * PoseCorrector never touches the heading.)
     */
    private Pose2 drifted(double dx, double dy) {
        return new Pose2(truth.x + dx, truth.y + dy, truth.heading);
    }

    /** How far an odometry position is from the truth, in inches. */
    private double errorOf(Pose2 odometry) {
        return Math.hypot(odometry.x - truth.x, odometry.y - truth.y);
    }

    // ------------------------------------------------------------------ the tests

    /**
     * Situation: the feature is switched off (the shipped default) and odometry has drifted by (3, -2) in.
     * Expected: update returns null (no correction) and nothing is counted as applied.
     * Why: until the camera mount is verified on the real robot, a wrong mount would pull the position the wrong way,
     *     so the default must be harmless.
     */
    @Test
    public void doesNothingWhileSwitchedOff() {
        config.enabled = false;
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(3, -2);
        assertNull(corrector.update(1.0, odo, 0, 0, freshFrame(1.0), viewsFor(odo, true, ALL_FOUR)));
        assertEquals(0, corrector.appliedCount());
    }

    /**
     * Situation: odometry is 5 in off (4, -3), inside the 10 in gate; the robot stands still and a fresh, clean
     *     sighting arrives every 0.25 s (the minimum interval) for 25 tries.
     * Expected: every correction that is returned makes the error smaller; by the end the error is under 0.1 in; the
     *     heading is never touched.
     * Why: this is the whole point of the feature: a slow, steady pull toward the truth. Each step removes a fixed
     *     fraction (the gain) of what is left, so the error shrinks geometrically.
     */
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
        // 0.1 in is about 2.5 mm, far finer than a shot needs. 25 tries leave plenty of slack to get there.
        assertTrue("error should be nearly gone, was " + previousError, previousError < 0.1);
        // 1e-12 radians: the heading must come out unchanged; only rounding is tolerated.
        assertEquals("the heading is never touched", truth.heading, odo.heading, 1e-12);
    }

    /**
     * Situation: (a) odometry 2 in off; (b) odometry 9 in off (still inside the 10 in gate).
     * Expected: (a) the step is exactly gain x 2 in and points back toward the truth (dx is negative because the drift
     *     was toward +x). (b) gain x 9 in would be bigger than the cap, so the step is capped at maxStepIn, and
     *     disagreementIn reports 9.0.
     * Why: one bad sighting must never move the position far in one go.
     */
    @Test
    public void oneCorrectionIsAFractionOfTheDisagreementAndNeverLargerThanTheStepLimit() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(2.0, 0.0);                           // 2 in: gain decides
        PoseCorrector.Correction small = corrector.update(1.0, odo, 0, 0, freshFrame(1.0), viewsFor(odo, true, ALL_FOUR));
        assertNotNull(small);
        // Small drift: the gain decides. 1e-6 in is rounding only.
        assertEquals(config.gain * 2.0, Math.hypot(small.dx, small.dy), 1e-6);
        assertTrue("the step points back toward the truth", small.dx < 0);

        PoseCorrector second = new PoseCorrector(config);
        Pose2 far = drifted(9.0, 0.0);                           // 9 in: the step limit decides
        PoseCorrector.Correction big = second.update(1.0, far, 0, 0, freshFrame(1.0), viewsFor(far, true, ALL_FOUR));
        assertNotNull(big);
        // Big drift: the step limit decides.
        assertEquals(config.maxStepIn, Math.hypot(big.dx, big.dy), 1e-6);
        assertEquals(9.0, big.disagreementIn, 1e-6);
    }

    /**
     * Situation: odometry is 5 in beyond the gate distance away from what the camera implies (15 in with the 10 in
     *     gate).
     * Expected: no correction; rejectedCount is 1 and appliedCount is 0; the status text says 'ignored'.
     * Why: a disagreement that large is far more likely a wrong camera reading or a wrong mount than drift. Trusting it
     *     would drag the robot across the field.
     */
    @Test
    public void aSightingThatDisagreesByMoreThanTheGateIsThrownAway() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(config.gateIn + 5.0, 0.0);
        assertNull(corrector.update(1.0, odo, 0, 0, freshFrame(1.0), viewsFor(odo, true, ALL_FOUR)));
        assertEquals(0, corrector.appliedCount());
        assertEquals(1, corrector.rejectedCount());
        assertTrue(corrector.status(), corrector.status().contains("ignored"));
    }

    /**
     * Situation: the same 2 in drift, reported first while moving 1 in/s over the speed limit, then while turning
     *     10 degrees/s over the turn limit, then while standing still.
     * Expected: the first two are ignored (null); the third is corrected.
     * Why: the camera image is always a little old (0.05 s in these tests). If the robot is moving, the picture shows
     *     where it WAS, so correcting from it would add error instead of removing it.
     */
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

    /**
     * Situation: (1) an image older than the maximum age, (2) a frame that is not new (the same image seen again), (3)
     *     a camera that is not connected.
     * Expected: no correction in any of the three, and nothing is applied.
     * Why: only a fresh, real image says anything about the present position. Re-using one image would apply the same
     *     correction twice.
     */
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

    /**
     * Situation: a correction is applied, then another good sighting arrives half the minimum interval later, then one
     *     just after the full interval.
     * Expected: the early one is ignored; the later one is applied.
     * Why: writing the position to the Pinpoint costs a transaction on the I2C wiring bus, so corrections are
     *     rate-limited. Spacing them out also keeps the pull smooth.
     */
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

    /**
     * Situation: (1) only one tag visible; (2) all four visible but the cell is tipped DOWN; (3) two tags (the outer
     *     ones, 0 and 3) visible.
     * Expected: (1) and (2) give no correction; (3) is corrected.
     * Why: one tag gives a poor position fix. A DOWN cell's tags are not where the UP table says, so the implied
     *     position would be wrong. Two tags are the minimum the settings ask for.
     */
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

    /**
     * Situation: the camera sees no cluster at all (an empty list).
     * Expected: no correction, and the status text says 'no usable cluster'.
     * Why: the corrector must cope quietly with an empty view. The status line goes on the driver's screen and says why
     *     nothing happened.
     */
    @Test
    public void whenNoTagIsInViewNothingHappens() {
        PoseCorrector corrector = new PoseCorrector(config);
        Pose2 odo = drifted(2.0, 0.0);
        assertNull(corrector.update(1.0, odo, 0, 0, freshFrame(1.0), new ArrayList<ClusterView>()));
        assertTrue(corrector.status(), corrector.status().contains("no usable cluster"));
    }
}
