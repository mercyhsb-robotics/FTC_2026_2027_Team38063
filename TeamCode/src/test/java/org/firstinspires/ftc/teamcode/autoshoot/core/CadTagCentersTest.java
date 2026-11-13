package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Pins the field model's AprilTag cluster geometry to the centers measured in the official field CAD (field_cad.md section 5.2, all 16
 * tags, both HIVE states), and to the mechanics that produce them: a tip turns a cell exactly 60 degrees about the pivot axle, the
 * cell's base, opening and sticker are tilted 30 degrees, and the cluster origin (the aim point) lies where the SDK database puts it
 * relative to the tag row.
 *
 * <p>How it works: no robot. The table below is the CAD's tag-center table in the wall-corner frame (x, then y and height z for the state
 * at the start of the match, then y and z after one tip). For every tag the model must reproduce the centers to 0.02 in. The CAD
 * itself is symmetric between the alliances to 0.017 in (the HIVE parts are not placed exactly half a turn apart), so 0.02 in is the
 * tightest tolerance that is not tighter than the source.
 *
 * <p>Words used: <b>state A</b> is the match-start state (the red audience cell and the blue rear cell are UP), <b>state B</b> is after
 * one tip of that HIVE; <b>tag row</b> is the line of the four tag centers on a sticker; <b>UP row</b> and <b>DOWN row</b> are the
 * rows of a cell that is up or down.
 */
public class CadTagCentersTest {
    /** The CAD's own alliance symmetry error is 0.017 in, so nothing tighter than this can be asked of any number copied from it. */
    private static final double CAD_TOLERANCE_IN = 0.02;
    /** The mechanics (the 60 degree turn, the 30 degree tilt) are exact; the constants are rounded to three decimals. */
    private static final double ROUNDING_IN = 0.002;

    /** {tag ID, x, y in state A, z in state A, y in state B, z in state B}: field_cad_tags.csv, W-frame columns. */
    private static final double[][] CAD_TAGS = {
            {30, 64.432, 82.285, 35.522, 83.778, 49.791},
            {31, 60.682, 82.285, 35.522, 83.778, 49.791},
            {32, 55.182, 82.285, 35.522, 83.778, 49.791},
            {33, 51.432, 82.285, 35.522, 83.778, 49.791},
            {34, 51.432, 57.570, 49.791, 59.063, 35.522},
            {35, 55.182, 57.570, 49.791, 59.063, 35.522},
            {36, 60.682, 57.570, 49.791, 59.063, 35.522},
            {37, 64.432, 57.570, 49.791, 59.063, 35.522},
            {38, 76.932, 59.063, 35.522, 57.570, 49.791},
            {39, 80.682, 59.063, 35.522, 57.570, 49.791},
            {40, 86.182, 59.063, 35.522, 57.570, 49.791},
            {41, 89.932, 59.063, 35.522, 57.570, 49.791},
            {42, 89.932, 83.778, 49.791, 82.285, 35.522},
            {43, 86.182, 83.778, 49.791, 82.285, 35.522},
            {44, 80.682, 83.778, 49.791, 82.285, 35.522},
            {45, 76.932, 83.778, 49.791, 82.285, 35.522}};

    private final FieldModel field = new FieldModel();

    /** The model's center of tag {@code id} as {x, y, z} in the match-start state (A) or after one tip (B). */
    private double[] modelCenter(int id, boolean stateA) {
        CellSpec cell = field.cellForTag(id);
        assertNotNull("tag " + id + " belongs to a cell", cell);
        boolean cellIsUpInStateA = field.initialUpSide(cell.alliance) == cell.side;
        boolean up = cellIsUpInStateA == stateA;
        return new double[] {cell.tagFieldX(cell.tagIndex(id)), up ? cell.tagRowY : cell.downTagRowY, up ? cell.tagRowZ : cell.downTagRowZ};
    }

    /**
     * Situation: all 16 tags in both HIVE states.
     * Expected: the model's x, y and height of every tag center are within 0.02 in of the CAD table.
     * Why: ClusterAnalyzer turns the camera's tag positions into a range and an implied robot position with these numbers, and tells UP
     *     from DOWN by the row's height.
     */
    @Test
    public void everyTagCenterMatchesTheCadInBothStates() {
        assertEquals(16, CAD_TAGS.length);
        for (double[] t : CAD_TAGS) {
            int id = (int) t[0];
            double[] a = modelCenter(id, true);
            double[] b = modelCenter(id, false);
            assertEquals("tag " + id + " x", t[1], a[0], CAD_TOLERANCE_IN);
            assertEquals("tag " + id + " y at match start", t[2], a[1], CAD_TOLERANCE_IN);
            assertEquals("tag " + id + " z at match start", t[3], a[2], CAD_TOLERANCE_IN);
            assertEquals("tag " + id + " x after a tip", t[1], b[0], CAD_TOLERANCE_IN);
            assertEquals("tag " + id + " y after a tip", t[4], b[1], CAD_TOLERANCE_IN);
            assertEquals("tag " + id + " z after a tip", t[5], b[2], CAD_TOLERANCE_IN);
        }
    }

    /**
     * Situation: a tag center that is wrong by a tenth of an inch, as it would be if a row height were copied from the OLD report
     *     (UP row 49.85, DOWN row 35.58 against the CAD's 49.791 and 35.522, aim point 58.31 against 58.254).
     * Expected: the comparison above would notice: the old numbers are off by more than the 0.02 in tolerance (0.059, 0.058 and 0.056 in).
     * Why: it shows the pin is tight enough to catch the very numbers this step replaced.
     */
    @Test
    public void theOldReportsRowHeightsWouldNotPassThePin() {
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        assertTrue(Math.abs(49.85 - aud.tagRowZ) > CAD_TOLERANCE_IN);
        assertTrue(Math.abs(35.58 - aud.downTagRowZ) > CAD_TOLERANCE_IN);
        assertTrue(Math.abs(58.31 - aud.originZ) > CAD_TOLERANCE_IN);
        assertEquals(49.791, aud.tagRowZ, ROUNDING_IN);
    }

    /**
     * Situation: the DOWN row computed from the UP row by turning it 60 degrees about the pivot axle (y = the field center, height 43.95 in).
     * Expected: it is the DOWN row of the model (11.611 in out, 35.522 in up) for the audience cell and, mirrored, for the rear cell.
     * Why: this is the mechanics of a tip. If either row were mistyped, the rotation would no longer give the other.
     */
    @Test
    public void aTipTurnsTheUpRowIntoTheDownRowAboutThePivotAxle() {
        double tip = Math.toRadians(BioBuzzConstants.Field.HIVE_TIP_ANGLE_DEG);
        assertEquals(60.0, BioBuzzConstants.Field.HIVE_TIP_ANGLE_DEG, 0.0);
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        // coordinates relative to the pivot axle: horizontal distance toward the audience (positive) and height above the axle
        double out = field.center() - aud.tagRowY;
        double up = aud.tagRowZ - BioBuzzConstants.Field.HIVE_PIVOT_Z_IN;
        double downOut = out * Math.cos(tip) + up * Math.sin(tip);
        double downUp = -out * Math.sin(tip) + up * Math.cos(tip);
        assertEquals("DOWN row out from the pivot plane", field.center() - aud.downTagRowY, downOut, ROUNDING_IN);
        assertEquals("DOWN row height", aud.downTagRowZ, BioBuzzConstants.Field.HIVE_PIVOT_Z_IN + downUp, ROUNDING_IN);
        // the rotation keeps the distance to the axle: the tag row swings on an arc of radius 14.35 in
        assertEquals(Math.hypot(out, up), Math.hypot(field.center() - aud.downTagRowY, aud.downTagRowZ - BioBuzzConstants.Field.HIVE_PIVOT_Z_IN), ROUNDING_IN);
        // the rear cell is the mirror image about the pivot plane
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        assertEquals(aud.tagRowY - field.center(), field.center() - rear.tagRowY, ROUNDING_IN);
        assertEquals(aud.downTagRowY - field.center(), field.center() - rear.downTagRowY, ROUNDING_IN);
        assertEquals(aud.downTagRowZ, rear.downTagRowZ, ROUNDING_IN);
    }

    /**
     * Situation: the cluster origin (the aim point) computed from the tag row with the 30 degree tilt of the cell, the SDK's 7.1874 in
     *     between the tag row and the front of the cell and its 5.622 in from the sticker plane to the origin.
     * Expected: 16.518 in out and 58.254 in up, the constants, to 0.002 in.
     * Why: the tilt is how a flat sticker on a tilted base turns into the aim point at the middle of the opening; it ties the two heights
     *     and the two distances together.
     */
    @Test
    public void theAimPointFollowsFromTheTagRowAndTheThirtyDegreeTilt() {
        double tilt = Math.toRadians(BioBuzzConstants.Field.CELL_TILT_DEG);
        assertEquals(30.0, BioBuzzConstants.Field.CELL_TILT_DEG, 0.0);
        double out = BioBuzzConstants.Field.UP_CELL_TAG_ROW_OUT_IN + BioBuzzConstants.Field.TAG_ROW_BEHIND_CELL_FRONT_IN * Math.cos(tilt)
                - BioBuzzConstants.Field.CLUSTER_ORIGIN_ABOVE_STICKER_IN * Math.sin(tilt);
        double z = BioBuzzConstants.Field.UP_CELL_TAG_ROW_Z_IN + BioBuzzConstants.Field.TAG_ROW_BEHIND_CELL_FRONT_IN * Math.sin(tilt)
                + BioBuzzConstants.Field.CLUSTER_ORIGIN_ABOVE_STICKER_IN * Math.cos(tilt);
        assertEquals(BioBuzzConstants.Field.UP_CELL_ORIGIN_OUT_IN, out, ROUNDING_IN);
        assertEquals(BioBuzzConstants.Field.UP_CELL_ORIGIN_Z_IN, z, ROUNDING_IN);
        // the same in the model: aim point minus tag row, as ClusterAnalyzer uses it
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        assertEquals(3.414, aud.tagRowY - aud.originY, ROUNDING_IN);
    }

    /**
     * Situation: which way the tags face (normal 0, -0.5, -0.866 for the red audience cell when it is UP: 60 degrees below horizontal,
     *     toward the audience wall).
     * Expected: the horizontal part of the facing is the direction the opening faces: -y for the audience cell; after a tip it is the other
     *     way; the angle below horizontal is 90 - 30 = 60 degrees.
     * Why: a camera has to be on the front side of the sticker plane to see the tags; the code only aims at a cell from the side its
     *     opening faces, which is that side.
     */
    @Test
    public void theTagsFaceTheSideTheOpeningFacesWhileTheCellIsUp() {
        assertEquals(60.0, 90.0 - BioBuzzConstants.Field.CELL_TILT_DEG, 1e-9);
        double normalY = -Math.sin(Math.toRadians(BioBuzzConstants.Field.CELL_TILT_DEG));       // CAD: (0, -0.5, -0.866)
        double normalZ = -Math.cos(Math.toRadians(BioBuzzConstants.Field.CELL_TILT_DEG));
        assertEquals(-0.5, normalY, 1e-9);
        assertEquals(-0.866, normalZ, 0.001);
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        assertTrue("the audience cell's opening faces -y, like its tags while it is UP", Math.sin(aud.facingHeading) < 0 && normalY < 0);
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        assertTrue("the rear cell's opening faces +y", Math.sin(rear.facingHeading) > 0);
        // after a tip the cell hangs the other way: the audience cell's tags face +y, toward the pivot (CAD: (0, +0.5, -0.866))
        double normalYAfterATip = -normalY;
        assertEquals(0.5, normalYAfterATip, 1e-9);
        assertTrue(normalYAfterATip > 0 && Math.sin(aud.facingHeading) < 0);
    }

    /**
     * Situation: the cluster relations in {@link ConstantsCheck#checkClusterGeometry} fed the shipped numbers, then the same numbers with
     *     the aim point and a DOWN row left over from the old report.
     * Expected: the shipped numbers give no finding; the old aim point (58.31 in up) and the old DOWN row (35.58 in up) are each an ERROR.
     */
    @Test
    public void theConstantsCheckCatchesAClusterNumberLeftOverFromTheOldReport() {
        ConstantsCheck.Report ok = new ConstantsCheck.Report();
        check(ok, BioBuzzConstants.Field.UP_CELL_ORIGIN_Z_IN, BioBuzzConstants.Field.DOWN_CELL_TAG_ROW_Z_IN);
        assertTrue(ok.errors.toString(), ok.errors.isEmpty());
        ConstantsCheck.Report oldAim = new ConstantsCheck.Report();
        check(oldAim, 58.31, BioBuzzConstants.Field.DOWN_CELL_TAG_ROW_Z_IN);
        assertEquals(oldAim.errors.toString(), 1, oldAim.errors.size());
        assertTrue(oldAim.errors.get(0), oldAim.errors.get(0).contains("UP_CELL_ORIGIN"));
        ConstantsCheck.Report oldDown = new ConstantsCheck.Report();
        check(oldDown, BioBuzzConstants.Field.UP_CELL_ORIGIN_Z_IN, 35.58);
        assertEquals(oldDown.errors.toString(), 1, oldDown.errors.size());
        assertTrue(oldDown.errors.get(0), oldDown.errors.get(0).contains("DOWN_CELL_TAG_ROW"));
    }

    private static void check(ConstantsCheck.Report r, double originZ, double downZ) {
        ConstantsCheck.checkClusterGeometry(r, BioBuzzConstants.Field.UP_CELL_TAG_ROW_OUT_IN, BioBuzzConstants.Field.UP_CELL_TAG_ROW_Z_IN,
                BioBuzzConstants.Field.UP_CELL_ORIGIN_OUT_IN, originZ, BioBuzzConstants.Field.DOWN_CELL_TAG_ROW_OUT_IN, downZ,
                BioBuzzConstants.Field.HIVE_PIVOT_Z_IN, BioBuzzConstants.Field.HIVE_TIP_ANGLE_DEG, BioBuzzConstants.Field.CELL_TILT_DEG,
                BioBuzzConstants.Field.TAG_ROW_BEHIND_CELL_FRONT_IN, BioBuzzConstants.Field.CLUSTER_ORIGIN_ABOVE_STICKER_IN);
    }
}
