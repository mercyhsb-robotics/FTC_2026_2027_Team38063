package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The floor tiles: the official CAD lays them with a 23.5 in pitch (not 24 in), centered on the field, and the two outer tiles of
 * every row run to the walls. These tests pin the seams, the tile names and the places the rest of the code expects ("the center
 * of tile C1", "the start tile B1").
 *
 * <p>How it works: no robot. Every number in the expectations is written out by hand from the CAD table (the seams are at 23.674,
 * 47.174, 70.674, 94.174 and 117.674 in the wall-corner frame), so a change of the pitch or of the field size has to be made on
 * purpose.
 */
public class TileGridTest {
    private static final double EPS = 1e-9;

    /**
     * Situation: the seven seam positions along one axis, from the red (or audience) wall to the far wall.
     * Expected: 0, 23.674, 47.174, 70.674, 94.174, 117.674 and the field size 141.348; the middle seam is the field center.
     * Why: the CAD tiles are 23.5 in apart, so the seams are NOT at 24, 48, 72, 96, 120 as the manual's round numbers suggest.
     */
    @Test
    public void theSeamsAreTheCadTableInTheWallCornerFrame() {
        double[] expected = {0.0, 23.674, 47.174, 70.674, 94.174, 117.674, 141.348};
        assertEquals(expected.length, TileGrid.count() + 1);
        for (int k = 0; k < expected.length; k++) {
            assertEquals("seam " + k, expected[k], TileGrid.seamIn(k), 1e-9);
        }
        assertEquals("the middle seam is the field center", BioBuzzConstants.Field.CENTER_IN, TileGrid.seamIn(TileGrid.count() / 2), EPS);
    }

    /**
     * Situation: every pair of seams that are half-turn partners (k and 6 - k).
     * Expected: they add up to the field size, so the grid looks the same turned half a turn: red and blue see the same tiles.
     */
    @Test
    public void theGridIsHalfTurnSymmetric() {
        for (int k = 0; k <= TileGrid.count(); k++) {
            assertEquals("seams " + k + " and " + (TileGrid.count() - k), BioBuzzConstants.Field.SIZE_IN,
                    TileGrid.seamIn(k) + TileGrid.seamIn(TileGrid.count() - k), EPS);
        }
    }

    /**
     * Situation: the width of each tile along one axis.
     * Expected: the four inner tiles are exactly the 23.5 in pitch; the two outer ones are 23.674 in wide (they run to the walls).
     */
    @Test
    public void innerTilesAreThePitchWideAndOuterTilesRunToTheWalls() {
        for (int k = 0; k < TileGrid.count(); k++) {
            double width = TileGrid.seamIn(k + 1) - TileGrid.seamIn(k);
            boolean outer = k == 0 || k == TileGrid.count() - 1;
            assertEquals("tile " + k, outer ? 23.674 : BioBuzzConstants.Field.TILE_PITCH_IN, width, 1e-9);
        }
    }

    /**
     * Situation: points just either side of the seams, and the places the code talks about.
     * Expected: x = 23.8 is already tile B (a 24 in grid would say A) and x = 47.0 is still B (a 24 in grid would say B too, but
     *     47.3 is C where a 24 in grid says B); the corners are A1 and F6; the start tiles of the catalog, B1 and C1, hold their x.
     * Why: the tile name in a route and in a diagram must agree with where the robot really stands on the CAD field.
     */
    @Test
    public void tileNamesFollowTheCadSeamsNotATwentyFourInchGrid() {
        assertEquals("A1", TileGrid.nameAt(0.0, 0.0));
        assertEquals("F6", TileGrid.nameAt(BioBuzzConstants.Field.SIZE_IN, BioBuzzConstants.Field.SIZE_IN));
        assertEquals("A1", TileGrid.nameAt(23.6, 5.0));
        assertEquals("B1", TileGrid.nameAt(23.8, 5.0));                // a 24 in grid would still say A1
        assertEquals("B1", TileGrid.nameAt(47.0, 5.0));
        assertEquals("C1", TileGrid.nameAt(47.3, 5.0));                // a 24 in grid would still say B1
        assertEquals("A5", TileGrid.nameAt(5.0, 95.0));                // a 24 in grid says A4: the LOADING ZONE tiles are rows 5 on the CAD grid
        assertEquals("B1", TileGrid.nameAt(35.0, BioBuzzConstants.Robot.FRONT_EXTENT_IN));
        assertEquals("C1", TileGrid.nameAt(58.0, BioBuzzConstants.Robot.FRONT_EXTENT_IN));
        assertEquals("A3", TileGrid.nameAt(9.2112, 70.2702));          // the LeavePark start: row 3 runs from 47.174 to 70.674
        assertEquals("E6", TileGrid.nameAt(BioBuzzConstants.Field.SIZE_IN - 35.0, BioBuzzConstants.Field.SIZE_IN - 5.0));
    }

    /**
     * Situation: tile boxes by name.
     * Expected: C1 is x 47.174 to 70.674, y 0 to 23.674; its center is 58.924 in along x, which is the TeleOp's default start x
     *     (the center of tile C1) and 1.0 in from the HIVE axis.
     */
    @Test
    public void tileBoxesAndCentersMatchTheDefaultStart() {
        Rect c1 = TileGrid.tile("C1");
        assertEquals(47.174, c1.minX, EPS);
        assertEquals(70.674, c1.maxX, EPS);
        assertEquals(0.0, c1.minY, EPS);
        assertEquals(23.674, c1.maxY, EPS);
        assertEquals(58.924, TileGrid.centerX(2), EPS);
        assertEquals("the TeleOp default start is the center of tile C1", TileGrid.centerX(2), BioBuzzConstants.Controls.DEFAULT_START_X_IN, EPS);
        assertEquals("C1", TileGrid.nameAt(BioBuzzConstants.Controls.DEFAULT_START_X_IN, BioBuzzConstants.Controls.DEFAULT_START_Y_IN));
        assertEquals(1.0, TileGrid.centerX(2) - BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, 1e-9);
        for (int col = 0; col < TileGrid.count(); col++) {
            for (int row = 0; row < TileGrid.count(); row++) {
                String name = TileGrid.name(col, row);
                assertEquals(name, name, TileGrid.nameAt(TileGrid.centerX(col), TileGrid.centerY(row)));
                Rect tile = TileGrid.tile(name);
                assertTrue(name, tile.contains(TileGrid.centerX(col), TileGrid.centerY(row)));
            }
        }
    }

    /** Situation: names that are not tiles. Expected: an IllegalArgumentException that says what a tile name is. */
    @Test
    public void aBadTileNameIsRefused() {
        for (String bad : new String[] {null, "", "C", "G1", "C7", "c1", "C10"}) {
            try {
                TileGrid.tile(bad);
                fail("'" + bad + "' is not a tile");
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains("tile"));
            }
        }
        try {
            TileGrid.seamIn(7);
            fail("there are only seams 0 to 6");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("seam"));
        }
    }
}
