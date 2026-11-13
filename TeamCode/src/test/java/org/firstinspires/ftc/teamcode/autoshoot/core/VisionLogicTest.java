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

public class VisionLogicTest {
    private static final double EPS = 1e-6;

    private final FieldModel field = new FieldModel();
    private final CameraModel camera = CameraModel.fromConstants();
    private final ClusterAnalyzer.Config cfg = ClusterAnalyzer.Config.fromConstants();
    private final ClusterAnalyzer analyzer = new ClusterAnalyzer(field, cfg);

    // ------------------------------------------------------------------ a virtual camera

    /** Where tag {@code index} of a cell is in the field for a cell that is UP (row y/z) or DOWN. */
    private double[] tagWorld(CellSpec cell, int index, boolean cellIsUp) {
        double x = cell.tagFieldX(index);
        if (cellIsUp) {
            return new double[]{x, cell.tagRowY, cell.tagRowZ};
        }
        // A DOWN cell's tag row is 11.65 in out from the pivot plane on its own side, 35.58 in high.
        double y = field.center() + cell.side.ySign() * 11.65;
        return new double[]{x, y, cell.downTagRowZ};
    }

    /** What the camera would report for the chosen tags of a cell, seen from the given robot pose. */
    private List<TagObservation> observe(Pose2 robot, CellSpec cell, boolean cellIsUp, int[] indexes,
                                         double noiseIn, long seed) {
        Random rng = new Random(seed);
        List<TagObservation> out = new ArrayList<TagObservation>();
        for (int i : indexes) {
            double[] w = tagWorld(cell, i, cellIsUp);
            double f = robot.forwardOf(w[0], w[1]) + noiseIn * rng.nextGaussian();
            double l = robot.leftOf(w[0], w[1]) + noiseIn * rng.nextGaussian();
            double u = w[2] + noiseIn * rng.nextGaussian();
            double[] tt = camera.txTyOf(f, l, u);
            out.add(TagObservation.withPosition(cell.baseTagId + i, tt[0], tt[1], 0.5,
                    camera.azimuthOf(tt[0], tt[1]), f, l, u));
        }
        return out;
    }

    private static final int[] ALL = {0, 1, 2, 3};

    // ------------------------------------------------------------------ CameraModel

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

    @Test
    public void txOnARearCameraTurnsIntoABearingTowardTheRobotsLeft() {
        double straightBack = camera.azimuthOf(0, 0);
        assertEquals(Math.PI, Math.abs(straightBack), EPS);
        double a = camera.azimuthOf(10, 0);                      // tag right of the crosshair
        assertTrue("right-of-crosshair on a rear camera is back-and-left: " + Math.toDegrees(a),
                a > Math.PI / 2 && a < Math.PI);
    }

    @Test
    public void txScaleShrinksWithCameraPitchLikeTheCriticPredicted() {
        // A tag straight behind and 10 in to the side, elevated: a pitched camera sees less tx than the real bearing.
        CameraModel pitched = new CameraModel(0, 0, 0, 180, 50);
        double[] tt = pitched.txTyOf(-40, 10, 40);
        double realBearing = Math.toDegrees(Math.atan2(10, 40));
        assertTrue(Math.abs(tt[0]) < realBearing);
    }

    // ------------------------------------------------------------------ ClusterAnalyzer

    @Test
    public void anUpClusterGivesExactRangeBearingAndImpliedPositionFromAnyVisibleSubset() {
        Pose2 robot = Pose2.ofDegrees(55.0, 14.0, -97.0);
        CellSpec cell = field.cell(Alliance.RED, Side.AUDIENCE);
        double trueRange = ShotGeometry.rangeTo(robot, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
        double trueAimError = ShotGeometry.aimError(robot, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);

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

    private static CellSpec cell(CellSpec c) {
        return c;
    }

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
        assertTrue("worst range error " + worst, worst < 1.5);
    }

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
        // The cluster centre is 7.2 in to the robot's right here, so the shooter must turn to bring it behind.
        assertTrue(Math.abs(Math.toDegrees(v.aimErrorRad)) < 20);
    }

    // ------------------------------------------------------------------ HiveTracker

    private static VisionFrame frame(double now, boolean fresh) {
        return new VisionFrame(VisionFrame.Status.OK, fresh, 0.03, now, Collections.<TagObservation>emptyList());
    }

    private ClusterView view(CellSpec cell, ClusterView.Orientation o) {
        return new ClusterView(cell, 4, o, false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, false, Double.NaN, Double.NaN);
    }

    private HiveTracker tracker() {
        return new HiveTracker(field, Alliance.RED, HiveTracker.Config.fromConstants());
    }

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

    @Test
    public void aDownReadingOfOneCellMeansTheOtherCellIsUp() {
        HiveTracker t = tracker();
        List<ClusterView> says = Collections.singletonList(view(field.cell(Alliance.RED, Side.AUDIENCE), ClusterView.Orientation.DOWN));
        t.update(0.0, frame(0.0, true), says, Side.REAR);
        t.update(0.1, frame(0.1, true), says, Side.REAR);
        assertEquals(HiveTracker.Event.CHANGED_BY_VISION, t.update(0.2, frame(0.2, true), says, Side.REAR));
        assertEquals(Side.REAR, t.upSide());
    }

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

    @Test
    public void theDriverCanForceOrToggleTheBelief() {
        HiveTracker t = tracker();
        assertEquals(HiveTracker.Event.CHANGED_BY_DRIVER, t.toggle());
        assertEquals(Side.REAR, t.upSide());
        assertFalse(t.confirmed());
        t.forceUp(Side.AUDIENCE);
        assertEquals(Side.AUDIENCE, t.upSide());
        assertEquals(Side.AUDIENCE, HiveTracker.sideOf(Pose2.ofDegrees(60, 20, 0), field));
        assertEquals(Side.REAR, HiveTracker.sideOf(Pose2.ofDegrees(60, 110, 0), field));
    }
}
