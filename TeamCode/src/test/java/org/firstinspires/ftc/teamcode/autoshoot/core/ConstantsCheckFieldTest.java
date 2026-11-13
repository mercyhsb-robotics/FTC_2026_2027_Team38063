package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link ConstantsCheck#checkFieldNumbers} refuses field numbers that cannot belong to one field: a tile pitch that does not fit
 * the field, a HIVE axis outside the red half of its frame, a FLOWER that is not against a wall, a LOADING ZONE that does not
 * touch the red wall. The shipped CAD numbers must pass it, and a number left over from the old 144 in frame must fail it.
 *
 * <p>How it works: no robot. The check takes its numbers as parameters, so each test starts from the shipped numbers and changes
 * one of them; the first test shows that the shipped numbers alone give no finding.
 */
public class ConstantsCheckFieldTest {
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;

    /** The shipped numbers, run through the check with {@code size} and {@code axis} replaced. */
    private static ConstantsCheck.Report check(double size, double pitch, int tiles, double axis, double[][] flowers, double xMin, double xMax) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkFieldNumbers(r, size, pitch, tiles, BioBuzzConstants.Field.HIVE_FRAME_WIDTH_X_IN,
                BioBuzzConstants.Field.HIVE_FRAME_DEPTH_Y_IN, axis, BioBuzzConstants.Field.CELL_OPENING_WIDTH_IN, flowers,
                BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN, xMin, xMax, BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN,
                BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MAX_IN);
        return r;
    }

    private static ConstantsCheck.Report shipped() {
        return check(SIZE, BioBuzzConstants.Field.TILE_PITCH_IN, BioBuzzConstants.Field.TILES_PER_SIDE, BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN,
                BioBuzzConstants.Field.FLOWER_CENTERS_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN);
    }

    /** Situation: the shipped CAD numbers. Expected: no finding at all. */
    @Test
    public void theShippedFieldNumbersFitEachOther() {
        assertTrue(shipped().errors.toString(), shipped().errors.isEmpty());
    }

    /**
     * Situation: the old 144 in size with the CAD FLOWER centers, which are measured 2.634 in from walls 141.348 in apart.
     * Expected: an ERROR naming a FLOWER, because the blue-wall and rear-wall FLOWERS are now 5.3 in from the (too distant) walls.
     * Why: this is the mistake the frame change is meant to catch: one number set in the old frame and the others in the new one.
     */
    @Test
    public void aFieldSizeLeftOverFromTheOldFrameBreaksTheFlowers() {
        ConstantsCheck.Report r = check(144.0, 24.0, 6, BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, BioBuzzConstants.Field.FLOWER_CENTERS_IN,
                BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN);
        assertEquals(r.errors.toString(), 2, r.errors.size());
        for (String error : r.errors) {
            assertTrue(error, error.contains("FLOWER_CENTERS_IN"));
        }
    }

    /**
     * Situation: tiles that do not fit the field: 24 in tiles in the 141.348 in field (144 in of tile), and 20 in tiles (120 in).
     * Expected: an ERROR that names the tile pitch both times.
     */
    @Test
    public void tilesThatDoNotFillTheFloorAreRefused() {
        for (double pitch : new double[] {24.0, 20.0}) {
            ConstantsCheck.Report r = check(SIZE, pitch, 6, BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, BioBuzzConstants.Field.FLOWER_CENTERS_IN,
                    BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN);
            assertEquals(r.errors.toString(), 1, r.errors.size());
            assertTrue(r.errors.get(0), r.errors.get(0).contains("TILE_PITCH_IN"));
        }
    }

    /**
     * Situation: a red HIVE axis typed in the wrong half: 66.0 in, which puts the 20 in opening across the center line.
     * Expected: an ERROR that names the axis.
     */
    @Test
    public void aHiveAxisOutsideTheRedHalfOfTheFrameIsRefused() {
        ConstantsCheck.Report r = check(SIZE, BioBuzzConstants.Field.TILE_PITCH_IN, BioBuzzConstants.Field.TILES_PER_SIDE, 66.0,
                BioBuzzConstants.Field.FLOWER_CENTERS_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN);
        assertEquals(r.errors.toString(), 1, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains("RED_HIVE_AXIS_X_IN"));
    }

    /**
     * Situation: a FLOWER center typed at 2.4 in from the wall (the manual's old figure) instead of the CAD's 2.634 in, and a
     *     LOADING ZONE whose near edge is the CAD tape (0.574 in off the wall) instead of the wall.
     * Expected: one ERROR for the FLOWER and one for the zone.
     */
    @Test
    public void aFlowerOffItsWallAndAZoneOffTheWallAreRefused() {
        double[][] flowers = {{2.4, 47.281}, {47.281, 138.714}, {138.714, 94.067}, {94.067, 2.634}};
        ConstantsCheck.Report r = check(SIZE, BioBuzzConstants.Field.TILE_PITCH_IN, BioBuzzConstants.Field.TILES_PER_SIDE,
                BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, flowers, 0.574, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN);
        assertEquals(r.errors.toString(), 2, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains("FLOWER_CENTERS_IN[0]"));
        assertTrue(r.errors.get(1), r.errors.get(1).contains("LOADING ZONE"));
    }
}
