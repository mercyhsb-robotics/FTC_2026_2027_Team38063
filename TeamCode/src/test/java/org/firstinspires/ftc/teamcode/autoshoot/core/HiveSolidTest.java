package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The exact HIVE solid below the robot's height ({@link HiveSolid}): two foot bars and four leg stubs, with the open lane between them,
 * compared with what the official field CAD measured: a robot 12.13 in tall can drive north-south under the HIVE anywhere in a lane
 * 40.1 in wide (x 51.94 to 92.07 in the centre-72 numbering used by the CAD report, 50.61 to 90.74 in the wall-corner frame the code
 * uses), 45.5 in wide between the stubs; for a 29 in tall robot the lane is 30.9 in wide.
 *
 * <p>How it works: no robot. The shapes are built for several heights and compared with the CAD numbers; the 13 corners of the CAD's plan
 * "shadow" of everything lower than 12.13 in must lie on the model. The model is allowed to be a little bigger than the CAD (it is for
 * planning: a lane that is too narrow is safe, one that is too wide is not), never smaller.
 *
 * <p>Words used: <b>stub</b> is the part of a leg that is lower than the robot; <b>lane</b> is the open floor between the left pieces and the
 * right pieces; <b>wall-corner frame</b> is the frame of the code (origin at the inner corner of the red and audience walls); the CAD report
 * uses a frame centered on the field, 1.326 in different in x and y.
 */
public class HiveSolidTest {
    private static final double EPS = 1e-9;
    /** The CAD report's numbering is the wall-corner frame plus this much in x and y. */
    private static final double CAD_FRAME_SHIFT_IN = 1.326;
    private static final double CENTER = BioBuzzConstants.Field.CENTER_IN;

    private static HiveSolid solid(double height) {
        return HiveSolid.of(CENTER, height);
    }

    /**
     * Situation: the solid for the robot's own height limit (13.0 in), and for a robot lower than the foot of a leg (0.3 in).
     * Expected: 6 pieces (2 bars + 4 stubs) for the first, 2 bars only for the second. The box around the pieces is the frame footprint
     *     (49.466 x 38.944 in centered on the field), and the bars are 1.983 in wide and 38.944 in long.
     */
    @Test
    public void twoBarsAndFourStubsInsideTheFrameFootprint() {
        HiveSolid s = solid(BioBuzzConstants.Auto.HIVE_SOLID_HEIGHT_IN);
        assertEquals(6, s.pieces().size());
        assertEquals(2, solid(0.3).pieces().size());
        Rect frame = new FieldModel().hiveFrame();
        assertEquals(frame.minX, s.envelope().minX, EPS);
        assertEquals(frame.maxX, s.envelope().maxX, EPS);
        assertEquals(frame.minY, s.envelope().minY, EPS);
        assertEquals(frame.maxY, s.envelope().maxY, EPS);
        for (int bar = 0; bar < 2; bar++) {
            Rect b = s.pieces().get(bar).bounds();
            assertEquals("bar width", 1.983, b.maxX - b.minX, 0.001);
            assertEquals("bar length", 38.944, b.maxY - b.minY, 0.001);
        }
        // the left bar's inner face is 22.75 in from the center line: 47.924 in from the red wall
        assertEquals(47.924, s.pieces().get(0).bounds().maxX, 1e-3);
        assertEquals(45.941, s.pieces().get(0).bounds().minX, 1e-3);
        assertTrue("the foot bars are taller than a 0.94 in ground clearance, so a robot cannot drive across them",
                BioBuzzConstants.Field.HIVE_FOOT_BAR_HEIGHT_IN > 0.94);
        // the pieces are smaller than the footprint box: the lane is not in them
        double area = 0.0;
        for (ConvexPolygon p : s.pieces()) {
            area += p.area();
        }
        assertTrue("pieces " + area + " in^2 against the box " + (frame.maxX - frame.minX) * (frame.maxY - frame.minY),
                area < 0.2 * (frame.maxX - frame.minX) * (frame.maxY - frame.minY));
    }

    /**
     * Situation: the lane of a robot 12.13 in tall.
     * Expected: it is x 50.61 to 90.74 where the leg stubs narrow it (40.1 in wide: the CAD report says 51.94 to 92.07 in its numbering) and
     *     47.92 to 93.42 between them (45.5 in wide, the CAD says 49.25 to 94.75). The model's lane is never WIDER than the CAD's (within 0.001 in) and
     *     at most 0.04 in narrower.
     */
    @Test
    public void theLaneOfA12InchRobotIsTheCadLane() {
        HiveSolid s = solid(12.13);
        double[] narrow = s.narrowestLane();
        double cadLeft = 51.94 - CAD_FRAME_SHIFT_IN;
        double cadRight = 92.07 - CAD_FRAME_SHIFT_IN;
        assertTrue("left tip " + narrow[0], narrow[0] >= cadLeft - 0.001 && narrow[0] <= cadLeft + 0.04);
        assertTrue("right tip " + narrow[1], narrow[1] <= cadRight + 0.001 && narrow[1] >= cadRight - 0.04);
        assertEquals("symmetric about the field center", 2.0 * CENTER, narrow[0] + narrow[1], EPS);
        assertEquals("about 40.1 in wide", 40.1, narrow[1] - narrow[0], 0.1);
        double[] middle = s.laneAtRow(CENTER);
        assertEquals(49.25 - CAD_FRAME_SHIFT_IN, middle[0], 0.001);
        assertEquals(94.75 - CAD_FRAME_SHIFT_IN, middle[1], 0.001);
        assertEquals("45.5 in wide between the bars", 45.5, middle[1] - middle[0], 0.001);
        // a row outside the frame has no lane
        assertNull(s.laneAtRow(CENTER + 30.0));
    }

    /**
     * Situation: the lane for taller robots: 18, 24 and 29 in.
     * Expected: the lane gets narrower as the legs lean in (the CAD: 37.0, 33.7 and 30.9 in wide); the model's lane is never wider than the CAD's
     *     and at most 0.12 in narrower on each side.
     */
    @Test
    public void aTallerRobotHasANarrowerLane() {
        double[][] cad = {{18, 53.53, 90.49}, {24, 55.18, 88.84}, {29, 56.54, 87.47}};
        double previousWidth = Double.POSITIVE_INFINITY;
        for (double[] row : cad) {
            double[] lane = solid(row[0]).narrowestLane();
            double left = row[1] - CAD_FRAME_SHIFT_IN;
            double right = row[2] - CAD_FRAME_SHIFT_IN;
            assertTrue(row[0] + " in, left " + lane[0], lane[0] >= left - 0.02 && lane[0] <= left + 0.12);
            assertTrue(row[0] + " in, right " + lane[1], lane[1] <= right + 0.02 && lane[1] >= right - 0.12);
            assertTrue("narrower than before", lane[1] - lane[0] < previousWidth);
            previousWidth = lane[1] - lane[0];
        }
    }

    /**
     * Situation: the corners of the CAD's own plan "shadow" of everything lower than 12.13 in (the report's 13-corner outline of the left
     *     component: foot bar plus two leg stubs), turned into the wall-corner frame; and the same outline mirrored for the right component.
     * Expected: every corner is on the model's solid (within 0.06 in).
     * Why: the solid must cover everything the CAD says the robot can hit. (The model may be bigger than the CAD; the tolerance is only for
     *     the corner coordinates being rounded to 0.01 in in the report.)
     */
    @Test
    public void theModelCoversTheCadShadow() {
        double[][] outline = {{47.27, 91.46}, {49.25, 90.37}, {51.83, 85.95}, {51.94, 84.85}, {50.91, 84.85}, {49.26, 87.52},
                {49.26, 56.48}, {50.91, 59.15}, {51.94, 59.15}, {51.83, 58.05}, {49.37, 54.04}, {49.25, 52.63}, {47.26, 52.55}};
        HiveSolid s = solid(12.13);
        for (double[] c : outline) {
            for (int mirror = 0; mirror < 2; mirror++) {
                double x = c[0] - CAD_FRAME_SHIFT_IN;
                double y = c[1] - CAD_FRAME_SHIFT_IN;
                if (mirror == 1) {
                    x = 2.0 * CENTER - x;
                }
                ConvexPolygon dot = ConvexPolygon.of(x - 0.001, y - 0.001, x + 0.001, y - 0.001, x + 0.001, y + 0.001, x - 0.001, y + 0.001);
                double nearest = Double.POSITIVE_INFINITY;
                for (ConvexPolygon p : s.pieces()) {
                    nearest = Math.min(nearest, p.signedGap(dot));
                }
                assertTrue("CAD corner (" + c[0] + ", " + c[1] + ") mirror " + mirror + " is " + nearest + " in off the model", nearest <= 0.06);
            }
        }
    }

    /**
     * Situation: the front and the rear stub of the left leg pair, 12.13 in tall.
     * Expected: the front stub's tip is at y 57.82 (the CAD outline says 59.15 in its numbering) and the rear one at y 83.53; the stubs are the
     *     only pieces in the lane rows near the two ends of the frame, and the middle of the frame is free of them.
     */
    @Test
    public void theStubsAreAtTheTwoEndsOfTheFrame() {
        HiveSolid s = solid(12.13);
        double frontTip = Double.NEGATIVE_INFINITY;
        double rearTip = Double.POSITIVE_INFINITY;
        for (int i = 2; i < s.pieces().size(); i++) {
            Rect b = s.pieces().get(i).bounds();
            if (b.maxY < CENTER) {
                frontTip = Math.max(frontTip, b.maxY);
            } else {
                rearTip = Math.min(rearTip, b.minY);
            }
        }
        assertEquals(59.15 - CAD_FRAME_SHIFT_IN, frontTip, 0.04);
        assertEquals(84.85 - CAD_FRAME_SHIFT_IN, rearTip, 0.04);
        assertEquals("symmetric", 2.0 * CENTER, frontTip + rearTip, EPS);
        // between the stubs the lane is the whole gap between the bars
        double[] between = s.laneAtRow(CENTER);
        assertEquals(CENTER - 22.75, between[0], EPS);
        assertEquals(CENTER + 22.75, between[1], EPS);
        // at a row through the front stubs it is narrower
        double[] atStub = s.laneAtRow(frontTip - 0.1);
        assertTrue(atStub[0] > between[0] + 2.0);
    }

    /**
     * Situation: the solid turned half a turn about the field center.
     * Expected: every piece of the turned solid is a piece of the solid (the frame carries both HIVEs and is its own half-turn image).
     */
    @Test
    public void theSolidIsItsOwnHalfTurnImage() {
        HiveSolid s = solid(13.0);
        for (ConvexPolygon turned : s.halfTurnedPieces()) {
            boolean found = false;
            for (ConvexPolygon p : s.pieces()) {
                Rect a = p.bounds();
                Rect b = turned.bounds();
                found |= Math.abs(a.minX - b.minX) < 1e-9 && Math.abs(a.maxX - b.maxX) < 1e-9
                        && Math.abs(a.minY - b.minY) < 1e-9 && Math.abs(a.maxY - b.maxY) < 1e-9 && Math.abs(p.area() - turned.area()) < 1e-9;
            }
            assertTrue("a piece without a half-turn partner: " + turned, found);
        }
    }

    /**
     * Situation: a robot (17.80 in wide, facing +y) in the middle of the lane at several rows, one pushed 8 in toward the red foot bar, and one
     *     standing on a foot bar.
     * Expected: the one in the middle has a positive gap to the solid (more than the 3 in lane margin), the pushed one is still clear but
     *     inside the margin (its edge is 2.9 in from the stub tip), and the one on the bar has a negative gap.
     */
    @Test
    public void theGapToTheSolidTellsTheLaneFromTheFeet() {
        HiveSolid s = solid(BioBuzzConstants.Auto.HIVE_SOLID_HEIGHT_IN);
        double margin = BioBuzzConstants.Auto.HIVE_LANE_MARGIN_IN;
        for (double y : new double[] {45.0, 55.0, 60.0, CENTER, 85.0, 95.0}) {
            double gap = s.signedGap(Footprint.of(Pose2.ofDegrees(CENTER, y, 90.0)));
            assertTrue("centered at y = " + y + ": gap " + gap, gap > margin);
        }
        // the narrowest row, the front stubs: the robot's red-side edge is 8.9 in left of its center
        double stubRow = 57.0;
        double centered = s.signedGap(Footprint.of(Pose2.ofDegrees(CENTER, stubRow, 90.0)));
        double pushed = s.signedGap(Footprint.of(Pose2.ofDegrees(CENTER - 8.0, stubRow, 90.0)));
        assertTrue(centered > margin);
        assertTrue("pushed " + pushed, pushed > 0.0 && pushed < margin);
        assertTrue("on the bar: " + s.signedGap(Footprint.of(Pose2.ofDegrees(CENTER - 22.0, CENTER, 90.0))),
                s.signedGap(Footprint.of(Pose2.ofDegrees(CENTER - 22.0, CENTER, 90.0))) < 0.0);
        assertNotNull(s.pieces());
        List<ConvexPolygon> pieces = s.pieces();
        assertEquals(6, pieces.size());
    }
}
