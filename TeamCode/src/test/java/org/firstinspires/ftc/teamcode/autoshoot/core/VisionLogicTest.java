package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.Test;

/**
 * Checks how the robot turns what the Limelight camera sees into facts it can act on: {@link CameraModel} (where the
 * camera sits and how to convert between camera and robot frames), {@link ClusterAnalyzer} (from AprilTags to 'is this
 * cell UP or DOWN, how far is the aim point, where am I standing?') and {@link HiveTracker} (the robot's belief about
 * which cell is UP, kept steady against noise).
 *
 * <p>How it works: there is no real camera. The helpers {@code tagWorld} and {@code observe} below are a 'virtual
 * camera': for a robot position we choose, they work out where each AprilTag would appear (its tx/ty angles and its
 * position in the robot's frame) and hand that to the real analyzer. Because we chose the position, we know the right
 * answer and can check it. The HiveTracker tests skip the camera and feed the tracker hand-made readings, one frame at
 * a time.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*VisionLogicTest"} to run only this
 * class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>AprilTag, cluster:</b> an AprilTag is a square black-and-white code the camera can read. The four tags
 *       on one HIVE cell are a 'cluster'.</li>
 *   <li><b>UP / DOWN:</b> whether a cell is tipped up (scorable) or down. The analyzer says UNKNOWN when it cannot
 *       tell.</li>
 *   <li><b>tx, ty:</b> where a tag appears in the camera image, in degrees: tx is how far right of the crosshair,
 *       ty how far above it.</li>
 *   <li><b>Azimuth, bearing:</b> the direction to something in the floor plan, as an angle from the robot's forward
 *       direction (counter-clockwise positive).</li>
 *   <li><b>Frame:</b> one camera snapshot. A frame is 'new' once and then repeats until the camera delivers the
 *       next one.</li>
 *   <li><b>Virtual camera:</b> test code that computes what a real camera WOULD report, so no camera is
 *       needed.</li>
 *   <li><b>Hysteresis:</b> the rules for changing a decision are stricter than the rules for keeping it, so one bad
 *       reading cannot flip it back and forth. HiveTracker needs several agreeing frames before it changes its
 *       mind.</li>
 *   <li><b>Tolerance / EPS (epsilon):</b> how far two decimal numbers may differ and still count as equal, to allow
 *       for rounding. Here one millionth.</li>
 *   <li><b>Fixture:</b> the shared setup for the tests (the fields below). JUnit builds a fresh copy for every
 *       test.</li>
 * </ul>
 */
public class VisionLogicTest {
    /** Allowed rounding error when comparing two decimal numbers: one millionth (inches or radians). */
    private static final double EPS = 1e-6;

    /**
     * The shared fixture (these four fields): the field map, the camera as mounted (from BioBuzzConstants), the
     * analyzer settings and the analyzer under test.
     */
    private final FieldModel field = new FieldModel();
    private final CameraModel camera = CameraModel.fromConstants();
    private final ClusterAnalyzer.Config cfg = ClusterAnalyzer.Config.fromConstants();
    private final ClusterAnalyzer analyzer = new ClusterAnalyzer(field, cfg);

    // ------------------------------------------------------------------ a virtual camera

    /**
     * Where tag {@code index} of a cell is in the field for a cell that is UP (row y/z) or DOWN.
     *
     * @return {x, y, z}: field x, field y and height above the floor, in inches
     */
    private double[] tagWorld(CellSpec cell, int index, boolean cellIsUp) {
        double x = cell.tagFieldX(index);
        if (cellIsUp) {
            return new double[]{x, cell.tagRowY, cell.tagRowZ};
        }
        // A DOWN cell's tag row is 11.65 in out from the pivot plane on its own side, 35.58 in high.
        double y = field.center() + cell.side.ySign() * 11.65;
        return new double[]{x, y, cell.downTagRowZ};
    }

    /**
     * What the camera would report for the chosen tags of a cell, seen from the given robot pose.
     *
     * <p>This is the virtual camera. For each tag it takes the tag's true field position, converts it to the robot's
     * frame (forward, left, height), optionally adds random noise, and then computes the tx/ty angles a real Limelight
     * would show, using the real CameraModel. It simplifies: no lens distortion, no tags missed or out of the camera's
     * view, the tag area is a fixed 0.5, and the noise is added to the 3D position, not to the image pixels.
     *
     * @param noiseIn standard deviation of the random noise, in inches (0 = perfect measurements)
     * @param seed picks the noise pattern; the same seed gives the same numbers, so a test is repeatable
     */
    private List<TagObservation> observe(Pose2 robot, CellSpec cell, boolean cellIsUp, int[] indexes,
                                         double noiseIn, long seed) {
        Random rng = new Random(seed);
        List<TagObservation> out = new ArrayList<TagObservation>();
        for (int i : indexes) {
            double[] w = tagWorld(cell, i, cellIsUp);
            // Add bell-curve noise (standard deviation noiseIn inches) to the tag's measured robot-frame position.
            double f = robot.forwardOf(w[0], w[1]) + noiseIn * rng.nextGaussian();
            double l = robot.leftOf(w[0], w[1]) + noiseIn * rng.nextGaussian();
            double u = w[2] + noiseIn * rng.nextGaussian();
            double[] tt = camera.txTyOf(f, l, u);
            out.add(TagObservation.withPosition(cell.baseTagId + i, tt[0], tt[1], 0.5,
                    camera.azimuthOf(tt[0], tt[1]), f, l, u));
        }
        return out;
    }

    /** The indexes of all four tags of a cell (the tags are numbered 0 to 3 in ID order). */
    private static final int[] ALL = {0, 1, 2, 3};

    // ------------------------------------------------------------------ CameraModel

    /**
     * Situation: 50 random points in the robot frame (a fixed seed, so the same 50 every run) are converted
     *     robot-to-camera and back again.
     * Expected: every point comes back to where it started, within 1e-6 in.
     * Why: if the two conversions are not exact inverses, every distance the camera reports would be off. 1e-6 in is
     *     floating-point rounding only.
     */
    @Test
    public void cameraAndRobotFramesRoundTrip() {
        Random rng = new Random(7);
        for (int k = 0; k < 50; k++) {
            double f = -80 + 160 * rng.nextDouble();
            double l = -40 + 80 * rng.nextDouble();
            double h = 5 + 60 * rng.nextDouble();
            double[] c = camera.robotToCamera(f, l, h);
            double[] r = camera.cameraToRobot(c[0], c[1], c[2]);
            assertEquals(f, r[0], EPS);
            assertEquals(l, r[1], EPS);
            assertEquals(h, r[2], EPS);
        }
    }

    /**
     * Situation: a camera at the robot center, 10 in up, looking out the BACK (yaw 180 degrees) and tilted up
     *     30 degrees.
     * Expected: a point 10 in to the image's right is 10 in to the robot's LEFT; a point 100 in along the optical axis
     *     (the line straight out of the lens) is behind the robot (-100 x cos 30 degrees) and up by 100 x sin
     *     30 degrees plus the camera height.
     * Why: our Limelight faces backward like the shooter, so left and right are swapped compared with a front camera.
     *     Getting that wrong would turn the robot the wrong way when it aligns.
     */
    @Test
    public void aRearCameraSeesImageRightAsRobotLeftAndTheOpticalAxisPointsBackAndUp() {
        CameraModel rear = new CameraModel(0, 0, 10, 180, 30);
        double[] right = rear.cameraToRobot(10, 0, 0);          // 10 in to the image's right
        assertEquals(0.0, right[0], EPS);
        assertEquals(10.0, right[1], EPS);                       // = robot LEFT
        double[] ahead = rear.cameraToRobot(0, 0, 100);          // along the optical axis
        assertEquals(-100 * Math.cos(Math.toRadians(30)), ahead[0], EPS);   // behind
        assertEquals(0.0, ahead[1], EPS);
        assertEquals(10 + 100 * Math.sin(Math.toRadians(30)), ahead[2], EPS); // and up
    }

    /**
     * Situation: the real camera mount from BioBuzzConstants; a tag exactly in the center of the image, then a tag
     *     10 degrees to the right of the crosshair.
     * Expected: the center tag is straight behind the robot (azimuth +180 or -180 degrees, either sign because of tiny
     *     rounding); the right-of-center tag is back-and-LEFT (between 90 and 180 degrees).
     * Why: this turns the camera's tx into a direction on the floor, which is what the aiming logic uses.
     */
    @Test
    public void txOnARearCameraTurnsIntoABearingTowardTheRobotsLeft() {
        double straightBack = camera.azimuthOf(0, 0);
        assertEquals(Math.PI, Math.abs(straightBack), EPS);
        double a = camera.azimuthOf(10, 0);                      // tag right of the crosshair
        assertTrue("right-of-crosshair on a rear camera is back-and-left: " + Math.toDegrees(a),
                a > Math.PI / 2 && a < Math.PI);
    }

    /**
     * Situation: a camera tilted up 50 degrees and a tag that is high up (40 in), 40 in behind and 10 in to the side.
     * Expected: tx, the sideways angle measured in the IMAGE, is smaller than the true floor bearing, atan(10/40) =
     *     14 degrees.
     * Why: when the camera looks steeply up, the same sideways distance covers fewer degrees of image width. The camera
     *     model must account for pitch or the aim error would be wrong. (The 'critic' in the name is a reviewer's
     *     earlier prediction of this effect.)
     */
    @Test
    public void txScaleShrinksWithCameraPitchLikeTheCriticPredicted() {
        // A tag straight behind and 10 in to the side, elevated: a pitched camera sees less tx than the real bearing.
        CameraModel pitched = new CameraModel(0, 0, 0, 180, 50);
        double[] tt = pitched.txTyOf(-40, 10, 40);
        double realBearing = Math.toDegrees(Math.atan2(10, 40));
        assertTrue(Math.abs(tt[0]) < realBearing);
    }

    // ------------------------------------------------------------------ ClusterAnalyzer

    /**
     * Situation: a robot at (55, 14) facing -97 degrees in front of the RED audience cell. The virtual camera shows any
     *     subset of that cell's four tags (all four, pairs, even a single tag) with zero noise.
     * Expected: exactly one view, for that cell, reading UP with full geometry; its range to the aim point, its aim
     *     error and the robot position it implies all equal the true values.
     * Why: this proves the geometry maths is right and that any subset of tags works, because tags can be hidden by
     *     other robots or leave the camera's view. 1e-6 is rounding error only, since there is no noise.
     */
    @Test
    public void anUpClusterGivesExactRangeBearingAndImpliedPositionFromAnyVisibleSubset() {
        Pose2 robot = Pose2.ofDegrees(55.0, 14.0, -97.0);
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        double trueRange = ShotGeometry.rangeTo(robot, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        double trueAimError = ShotGeometry.aimError(robot, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);

        // Every kind of partial view: all four tags, neighbouring pairs, the outer pair, and a single tag.
        int[][] subsets = {ALL, {0, 1}, {2, 3}, {1, 2}, {0, 3}, {3}};
        for (int[] subset : subsets) {
            List<ClusterView> views = analyzer.analyze(Alliance.RED, robot.heading,
                    observe(robot, cell, true, subset, 0, 1));
            assertEquals(1, views.size());
            ClusterView v = views.get(0);
            assertSame(cell, v.cell);
            assertEquals(ClusterView.Orientation.UP, v.orientation);
            assertTrue(v.hasGeometry);
            assertEquals("range, subset " + subset.length, trueRange, v.rangeIn, 1e-6);
            assertEquals("aim error", trueAimError, v.aimErrorRad, 1e-6);
            assertEquals("implied x", robot.x, v.impliedX, 1e-6);
            assertEquals("implied y", robot.y, v.impliedY, 1e-6);
        }
    }

    /**
     * Situation: a robot at (62, 124) facing 84 degrees, behind the RED rear cell, seeing all four of its tags.
     * Expected: the same results as on the audience side: reading UP, the implied position equals the true position,
     *     and the range is correct.
     * Why: the rear cluster's sticker is turned half a turn, so its tag IDs run the other way. This checks the code
     *     handles that.
     */
    @Test
    public void theFarSideClusterWorksTheSameWayFromTheRearOfTheHive() {
        Pose2 robot = Pose2.ofDegrees(62.0, 124.0, 84.0);
        CellSpec cell = field.cell(Alliance.RED, Side.REAR);
        List<ClusterView> views = analyzer.analyze(Alliance.RED, robot.heading, observe(robot, cell, true, ALL, 0, 2));
        assertEquals(1, views.size());
        ClusterView v = views.get(0);
        assertEquals(ClusterView.Orientation.UP, v.orientation);
        assertEquals(robot.x, v.impliedX, 1e-6);
        assertEquals(robot.y, v.impliedY, 1e-6);
        assertEquals(ShotGeometry.rangeTo(robot, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY),
                v.rangeIn, 1e-6);
    }

    /**
     * Situation: the BLUE robot (the half-turn of a red pose) sees BLUE's rear cluster and, in the same frame, an
     *     opponent (RED audience) cluster.
     * Expected: analyzing for BLUE returns only BLUE's cell, reading UP, with the true implied position; analyzing the
     *     same tags for RED returns exactly one view (the red cluster).
     * Why: only our own alliance's HIVE should drive decisions. The opponent's tags must be ignored.
     */
    @Test
    public void blueClustersAreHandledIdenticallyAndRedTagsAreIgnoredForBlue() {
        Pose2 red = Pose2.ofDegrees(55.0, 14.0, -97.0);
        Pose2 blue = Alliance.BLUE.fromRed(red, field.size());
        CellSpec blueCell = field.cell(Alliance.BLUE, Side.REAR);
        List<TagObservation> tags = observe(blue, blueCell, true, ALL, 0, 3);
        tags.addAll(observe(blue, field.cell(Alliance.RED, Side.AUDIENCE), true, ALL, 0, 4));   // an opponent cluster
        List<ClusterView> views = analyzer.analyze(Alliance.BLUE, blue.heading, tags);
        assertEquals(1, views.size());
        assertSame(blueCell, views.get(0).cell);
        assertEquals(ClusterView.Orientation.UP, views.get(0).orientation);
        assertEquals(blue.x, views.get(0).impliedX, 1e-6);
        assertEquals(blue.y, views.get(0).impliedY, 1e-6);
        // And the same call for RED sees only the red tags.
        assertEquals(1, analyzer.analyze(Alliance.RED, red.heading, tags).size());
    }

    /**
     * Situation: the robot is on the audience side; the RED rear cell is DOWN and its sticker is seen across the gap.
     * Expected: one view reading DOWN, with no geometry (no range or implied position is computed for a DOWN cell).
     * Why: a DOWN cell cannot be shot into. Its tags give themselves away twice: the IDs run right to left, and they
     *     are lower (about 36 in instead of 50 in).
     */
    @Test
    public void aDownClusterSeenAcrossTheGapReadsDownByOrderAndHeight() {
        // Robot on the audience side; the red REAR cell is DOWN, its sticker seen across the gap.
        Pose2 robot = Pose2.ofDegrees(59.0, 15.0, -90.0);
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        List<ClusterView> views = analyzer.analyze(Alliance.RED, robot.heading, observe(robot, rear, false, ALL, 0, 5));
        assertEquals(1, views.size());
        assertEquals(ClusterView.Orientation.DOWN, views.get(0).orientation);
        assertFalse(views.get(0).hasGeometry);
    }

    /**
     * Situation: only tag 1 of a cell is visible (once an UP cell, once a DOWN cell).
     * Expected: with no second tag to judge the left-to-right order, height alone decides: the UP tag reads UP and the
     *     DOWN tag reads DOWN.
     * Why: the camera often sees just one tag.
     */
    @Test
    public void aSingleTagIsClassifiedByItsHeightAlone() {
        Pose2 robot = Pose2.ofDegrees(59.0, 15.0, -90.0);
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        ClusterView up = analyzer.analyze(Alliance.RED, robot.heading, observe(robot, aud, true, new int[]{1}, 0, 6)).get(0);
        assertEquals(ClusterView.Orientation.UP, up.orientation);
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        ClusterView down = analyzer.analyze(Alliance.RED, robot.heading, observe(robot, rear, false, new int[]{1}, 0, 7)).get(0);
        assertEquals(ClusterView.Orientation.DOWN, down.orientation);
    }

    /**
     * Situation: tags whose left-to-right ORDER looks like an UP cell but whose measured HEIGHT is the DOWN height.
     * Expected: the reading is UNKNOWN.
     * Why: the two clues are independent. When they disagree something is wrong (a reflection, a bad 3D estimate), so
     *     the robot says 'unknown' instead of guessing. An UNKNOWN reading never counts as evidence that a cell is UP
     *     (rule G417: only launch into an UP cell).
     */
    @Test
    public void disagreementBetweenOrderAndHeightMeansUnknown() {
        Pose2 robot = Pose2.ofDegrees(59.0, 15.0, -90.0);
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        // Order says UP (correct IDs left to right) but the tags are at DOWN height.
        List<TagObservation> tags = observe(robot, aud, false, ALL, 0, 8);
        List<TagObservation> tweaked = new ArrayList<TagObservation>();
        for (TagObservation t : tags) {
            CellSpec c = field.cellForTag(t.id);
            double[] w = tagWorld(aud, c.tagIndex(t.id), true);
            double[] tt = camera.txTyOf(robot.forwardOf(w[0], w[1]), robot.leftOf(w[0], w[1]), w[2]);
            tweaked.add(TagObservation.withPosition(t.id, tt[0], tt[1], 0.5, camera.azimuthOf(tt[0], tt[1]),
                    robot.forwardOf(w[0], w[1]), robot.leftOf(w[0], w[1]), cell(aud).downTagRowZ));
        }
        ClusterView v = analyzer.analyze(Alliance.RED, robot.heading, tweaked).get(0);
        assertEquals(ClusterView.Orientation.UNKNOWN, v.orientation);
    }

    /** Returns the cell it is given (an identity helper; it only makes the line above read as 'the cell'). */
    private static CellSpec cell(CellSpec c) {
        return c;
    }

    /**
     * Situation: four tags whose measured height is 5 in, far from both the UP row (about 50 in) and the DOWN row
     *     (about 36 in) by more than TAG_HEIGHT_TOLERANCE_IN.
     * Expected: the reading is UNKNOWN.
     * Why: a tag that low is probably a reflection or a false detection. It must not be used to locate the robot or to
     *     approve a shot.
     */
    @Test
    public void implausibleTagHeightsAreRejected() {
        Pose2 robot = Pose2.ofDegrees(59.0, 15.0, -90.0);
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        List<TagObservation> tags = new ArrayList<TagObservation>();
        for (int i = 0; i < 4; i++) {
            double[] w = tagWorld(aud, i, true);
            double f = robot.forwardOf(w[0], w[1]);
            double l = robot.leftOf(w[0], w[1]);
            double[] tt = camera.txTyOf(f, l, 5.0);                    // 5 in high: nonsense (a reflection on the floor)
            tags.add(TagObservation.withPosition(aud.baseTagId + i, tt[0], tt[1], 0.5, camera.azimuthOf(tt[0], tt[1]), f, l, 5.0));
        }
        assertEquals(ClusterView.Orientation.UNKNOWN, analyzer.analyze(Alliance.RED, robot.heading, tags).get(0).orientation);
    }

    /**
     * Situation: 40 runs, each with different random noise (standard deviation 0.5 in) added to every tag's measured
     *     position.
     * Expected: every run still reads UP, and the worst range error over the 40 runs is under 1.5 in.
     * Why: real 3D tag positions are noisy, and the flywheel speed depends on the range, so the logic must still give a
     *     usable range. The fixed seeds 1 to 40 make the result the same on every run, so the test cannot fail at
     *     random.
     */
    @Test
    public void halfAnInchOfMeasurementNoiseGivesAnInchOfRangeAccuracy() {
        Pose2 robot = Pose2.ofDegrees(55.0, 14.0, -97.0);
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        double trueRange = ShotGeometry.rangeTo(robot, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        double worst = 0;
        for (long seed = 1; seed <= 40; seed++) {
            ClusterView v = analyzer.analyze(Alliance.RED, robot.heading, observe(robot, cell, true, ALL, 0.5, seed)).get(0);
            assertEquals(ClusterView.Orientation.UP, v.orientation);
            worst = Math.max(worst, Math.abs(v.rangeIn - trueRange));
        }
        // 1.5 in is three times the 0.5 in noise: a generous bound that averaging four tags normally beats.
        assertTrue("worst range error " + worst, worst < 1.5);
    }

    /**
     * Situation: a robot at (52, 14) facing -90 degrees sees all four tags of the RED audience cell, but they are
     *     stripped down to bearing only (angles tx/ty, no 3D position).
     * Expected: the reading is still UP (from the left-to-right tag order alone), hasGeometry is false (no range),
     *     hasBearing is true, and the aim error is under 20 degrees.
     * Why: if the camera gives only angles, the robot can still turn toward the HIVE even though it cannot measure the
     *     distance.
     */
    @Test
    public void bearingOnlyTagsStillGiveAnAimErrorButNoRange() {
        Pose2 robot = Pose2.ofDegrees(52.0, 14.0, -90.0);
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        List<TagObservation> full = observe(robot, cell, true, ALL, 0, 9);
        List<TagObservation> bearing = new ArrayList<TagObservation>();
        for (TagObservation t : full) {
            bearing.add(TagObservation.bearingOnly(t.id, t.txDeg, t.tyDeg, t.areaPct, t.azimuthRad));
        }
        // Without positions the orientation comes from the tag order alone.
        ClusterView v = analyzer.analyze(Alliance.RED, robot.heading, bearing).get(0);
        assertEquals(ClusterView.Orientation.UP, v.orientation);
        assertFalse(v.hasGeometry);
        assertTrue(v.hasBearing);
        // The cluster center (x = 59.25) is about 7 in to the robot's LEFT here (heading -90 degrees, so robot-left is
        // field +x; on the rear camera's image it appears on the right) and about 39 in behind the camera, so the true
        // bearing is about 10 degrees. The shooter must turn that much to point at it. 20 degrees is a loose bound.
        assertTrue(Math.abs(Math.toDegrees(v.aimErrorRad)) < 20);
    }

    // ------------------------------------------------------------------ HiveTracker

    /**
     * A camera snapshot with a healthy link, no tags and a 0.03 s old image, read at time {@code now}. {@code fresh}
     * says whether it is a NEW image (true) or a repeat of the previous one (false).
     */
    private static VisionFrame frame(double now, boolean fresh) {
        return new VisionFrame(VisionFrame.Status.OK, fresh, 0.03, now, Collections.<TagObservation>emptyList());
    }

    /**
     * A hand-made ClusterView that carries only the cell and the UP/DOWN/UNKNOWN reading (four tags, no geometry), so
     * the tracker tests need no camera simulation.
     */
    private ClusterView view(CellSpec cell, ClusterView.Orientation o) {
        return new ClusterView(cell, 4, o, false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, false, Double.NaN, Double.NaN);
    }

    /**
     * A fresh HiveTracker for the RED alliance with the real settings. It starts believing the AUDIENCE cell is up,
     * unconfirmed.
     */
    private HiveTracker tracker() {
        return new HiveTracker(field, Alliance.RED, HiveTracker.Config.fromConstants());
    }

    /**
     * Situation: a new tracker for RED, then one new frame in which the AUDIENCE cell reads UP.
     * Expected: before the frame it believes AUDIENCE is up (the match-start state) but unconfirmed; the matching
     *     sighting returns CONFIRMED and the tracker becomes confirmed, still on AUDIENCE.
     * Why: the controller will not feed until the camera (or the driver) has confirmed which cell is up (rule G417).
     */
    @Test
    public void theTrackerStartsFromTheSetupFigureAndGetsConfirmedByTheFirstSighting() {
        HiveTracker t = tracker();
        assertEquals(Side.AUDIENCE, t.upSide());
        assertFalse(t.confirmed());
        List<ClusterView> v = Collections.singletonList(view(field.cell(Alliance.RED, Side.AUDIENCE), ClusterView.Orientation.UP));
        assertEquals(HiveTracker.Event.CONFIRMED, t.update(0.1, frame(0.1, true), v, Side.AUDIENCE));
        assertTrue(t.confirmed());
        assertEquals(Side.AUDIENCE, t.upSide());
    }

    /**
     * Situation: the tracker believes AUDIENCE, but frames keep saying the REAR cell is UP: new frames at 1.00 s and
     *     1.05 s, a repeated (not new) frame at 1.06 s, and a new frame at 1.20 s.
     * Expected: nothing changes until the third new frame; then the result is CHANGED_BY_VISION, the belief is REAR, it
     *     is confirmed, and changeCount is 1.
     * Why: hysteresis: one bad frame must not send the robot to the far side of the field. The tracker wants
     *     HIVE_CONFIRM_FRAMES agreeing new frames spread over at least HIVE_CONFIRM_SEC.
     */
    @Test
    public void disagreementNeedsSeveralNewFramesAndTimeBeforeTheBeliefChanges() {
        HiveTracker t = tracker();
        List<ClusterView> says = Collections.singletonList(view(field.cell(Alliance.RED, Side.REAR), ClusterView.Orientation.UP));
        assertEquals(HiveTracker.Event.NONE, t.update(1.00, frame(1.00, true), says, Side.REAR));
        assertEquals(HiveTracker.Event.NONE, t.update(1.05, frame(1.05, true), says, Side.REAR));
        assertEquals(HiveTracker.Event.NONE, t.update(1.06, frame(1.06, false), says, Side.REAR));   // repeated frame: ignored
        assertEquals(Side.AUDIENCE, t.upSide());
        assertEquals(HiveTracker.Event.CHANGED_BY_VISION, t.update(1.20, frame(1.20, true), says, Side.REAR));
        assertEquals(Side.REAR, t.upSide());
        assertTrue(t.confirmed());
        assertEquals(1, t.changeCount());
    }

    /**
     * Situation: three new frames in which the AUDIENCE cell reads DOWN while the tracker believes AUDIENCE is up.
     * Expected: after the third frame (0.2 s after the first) the belief switches to REAR (CHANGED_BY_VISION).
     * Why: the two cells of a HIVE are always opposite, so 'this one is DOWN' means 'the other one is UP'.
     */
    @Test
    public void aDownReadingOfOneCellMeansTheOtherCellIsUp() {
        HiveTracker t = tracker();
        List<ClusterView> says = Collections.singletonList(view(field.cell(Alliance.RED, Side.AUDIENCE), ClusterView.Orientation.DOWN));
        t.update(0.0, frame(0.0, true), says, Side.REAR);
        t.update(0.1, frame(0.1, true), says, Side.REAR);
        assertEquals(HiveTracker.Event.CHANGED_BY_VISION, t.update(0.2, frame(0.2, true), says, Side.REAR));
        assertEquals(Side.REAR, t.upSide());
    }

    /**
     * Situation: ten new frames in which BOTH cells read UP (impossible).
     * Expected: no event at all; the belief stays AUDIENCE and stays unconfirmed.
     * Why: contradictory evidence is thrown away rather than guessed at.
     */
    @Test
    public void conflictingClustersInOneFrameAreIgnored() {
        HiveTracker t = tracker();
        List<ClusterView> both = new ArrayList<ClusterView>();
        both.add(view(field.cell(Alliance.RED, Side.AUDIENCE), ClusterView.Orientation.UP));
        both.add(view(field.cell(Alliance.RED, Side.REAR), ClusterView.Orientation.UP));
        for (int i = 0; i < 10; i++) {
            assertEquals(HiveTracker.Event.NONE, t.update(i * 0.05, frame(i * 0.05, true), both, Side.AUDIENCE));
        }
        assertEquals(Side.AUDIENCE, t.upSide());
        assertFalse(t.confirmed());
    }

    /**
     * Situation: the UP cluster is seen steadily for about 0.6 s; then every frame is empty while the robot stays on
     *     that side with a healthy camera.
     * Expected: within 0.8 s the tracker reports SUSPECTED_TIP, flips its belief to REAR and marks it unconfirmed.
     * Why: when a HIVE tips, its tags turn away and vanish. That is how the robot learns that the cell it was shooting
     *     into is no longer UP (rules G417 and G409).
     */
    @Test
    public void tagsSeenSteadilyThenLostFromTheUpSideSuggestATip() {
        HiveTracker t = tracker();
        List<ClusterView> seen = Collections.singletonList(view(field.cell(Alliance.RED, Side.AUDIENCE), ClusterView.Orientation.UP));
        double now = 0;
        for (int i = 0; i < 30; i++) {                 // 0.6 s of steady sighting
            now = i * 0.02;
            t.update(now, frame(now, true), seen, Side.AUDIENCE);
        }
        assertTrue(t.confirmed());
        HiveTracker.Event last = HiveTracker.Event.NONE;
        for (int i = 0; i < 40 && last == HiveTracker.Event.NONE; i++) {   // then nothing for up to 0.8 s
            now += 0.02;
            last = t.update(now, frame(now, true), Collections.<ClusterView>emptyList(), Side.AUDIENCE);
        }
        assertEquals(HiveTracker.Event.SUSPECTED_TIP, last);
        assertEquals(Side.REAR, t.upSide());
        assertFalse(t.confirmed());
    }

    /**
     * Situation: three trackers that each saw the UP cluster steadily and then lost it: (1) the robot is on the OTHER
     *     side of the HIVE, (2) the camera link has stalled, (3) the tags are missing for only 0.2 s and then come
     *     back.
     * Expected: no tip is suspected in any case; the belief stays AUDIENCE.
     * Why: tags also vanish for harmless reasons (we cannot see that cluster from the other side, no camera data, a
     *     one-frame glitch). Flipping the belief then would send the robot off to the wrong side.
     */
    @Test
    public void lossIsNotATipFromTheWrongSideOrWithADeadCameraOrAfterABriefGlitch() {
        List<ClusterView> seen = Collections.singletonList(view(field.cell(Alliance.RED, Side.AUDIENCE), ClusterView.Orientation.UP));
        List<ClusterView> none = Collections.<ClusterView>emptyList();

        HiveTracker wrongSide = tracker();
        double now = 0;
        for (int i = 0; i < 30; i++) {
            now = i * 0.02;
            wrongSide.update(now, frame(now, true), seen, Side.AUDIENCE);
        }
        for (int i = 0; i < 100; i++) {
            now += 0.02;
            assertEquals(HiveTracker.Event.NONE, wrongSide.update(now, frame(now, true), none, Side.REAR));
        }
        assertEquals(Side.AUDIENCE, wrongSide.upSide());

        HiveTracker deadCamera = tracker();
        now = 0;
        for (int i = 0; i < 30; i++) {
            now = i * 0.02;
            deadCamera.update(now, frame(now, true), seen, Side.AUDIENCE);
        }
        for (int i = 0; i < 100; i++) {
            now += 0.02;
            assertEquals(HiveTracker.Event.NONE, deadCamera.update(now, VisionFrame.unavailable(VisionFrame.Status.STALLED, now), none, Side.AUDIENCE));
        }
        assertEquals(Side.AUDIENCE, deadCamera.upSide());

        HiveTracker glitch = tracker();
        now = 0;
        for (int i = 0; i < 30; i++) {
            now = i * 0.02;
            glitch.update(now, frame(now, true), seen, Side.AUDIENCE);
        }
        for (int i = 0; i < 10; i++) {                 // 0.2 s without tags: shorter than the loss time
            now += 0.02;
            glitch.update(now, frame(now, true), none, Side.AUDIENCE);
        }
        now += 0.02;
        glitch.update(now, frame(now, true), seen, Side.AUDIENCE);
        assertEquals(Side.AUDIENCE, glitch.upSide());
    }

    /**
     * Situation: the driver toggles the belief (D-pad down); then the 'cell never seen' flip happens; then the driver
     *     forces a side. Finally the tracker is asked which side of the HIVE a pose is on.
     * Expected: toggle gives REAR with CHANGED_BY_DRIVER, and the driver's word counts as confirmed; flipBecauseUnseen
     *     goes back to AUDIENCE, unconfirmed; forceUp(AUDIENCE) keeps AUDIENCE. sideOf says AUDIENCE for y = 20 and
     *     REAR for y = 110 (the field center is y = 72).
     * Why: the driver can see the real HIVE, so their word wins. It is also the way forward when the camera is not
     *     working.
     */
    @Test
    public void theDriverCanForceOrToggleTheBelief() {
        HiveTracker t = tracker();
        assertEquals(HiveTracker.Event.CHANGED_BY_DRIVER, t.toggle());
        assertEquals(Side.REAR, t.upSide());
        assertTrue("the driver's word counts as confirmed", t.confirmed());
        t.flipBecauseUnseen();
        assertEquals(Side.AUDIENCE, t.upSide());
        assertFalse(t.confirmed());
        t.forceUp(Side.AUDIENCE);
        assertEquals(Side.AUDIENCE, t.upSide());
        assertEquals(Side.AUDIENCE, HiveTracker.sideOf(Pose2.ofDegrees(60, 20, 0), field));
        assertEquals(Side.REAR, HiveTracker.sideOf(Pose2.ofDegrees(60, 110, 0), field));
    }
}
