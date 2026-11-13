package org.firstinspires.ftc.teamcode.autoshoot.core;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The floor tiles of the field: where their edges ("seams") are and what each tile is called.
 *
 * <p>The floor is 6 x 6 foam tiles. Columns are lettered A to F from the red wall toward the blue wall (along x), rows are
 * numbered 1 to 6 from the audience wall toward the rear wall (along y), so A1 is the corner where the red wall meets the
 * audience wall, B1 is the next tile along the audience wall, and A2 is the next one along the red wall.
 *
 * <p><b>The tiles are NOT 24 in.</b> The official field CAD lays them with a pitch (center to center) of 23.5 in, and the field is
 * 141.348 in across between the walls ({@code BioBuzzConstants.Field.SIZE_IN}). The tile pattern is centered on the field, so
 * the middle seam is at the field center and the others follow every 23.5 in: x (or y) = 23.674, 47.174, 70.674, 94.174,
 * 117.674. The two outer tiles of every row are a little wider than 23.5 in (23.674 in) because they run to the walls. A
 * robot standing "in the middle of tile C1" is therefore at x = 58.924, not 60.
 *
 * <p>Used by the route diagrams (the tile pattern and its letters), by the checks that a route's start tile name matches its
 * start pose, and by the TeleOp's default start (the center of tile C1). Everything comes from the constants: nothing here
 * assumes a field size.
 */
public final class TileGrid {
    private TileGrid() {
    }

    /** The number of tiles along one side of the field (6). */
    public static int count() {
        return BioBuzzConstants.Field.TILES_PER_SIDE;
    }

    /**
     * The position, in inches along an axis, of seam number {@code k}. Seam 0 is the wall at 0 and seam {@link #count()} is the
     * wall at the far side; the seams between them are every {@code Field.TILE_PITCH_IN} either side of the field center.
     */
    public static double seamIn(int k) {
        if (k < 0 || k > count()) {
            throw new IllegalArgumentException("seam " + k + " is outside 0.." + count());
        }
        if (k == 0) {
            return 0.0;
        }
        if (k == count()) {
            return BioBuzzConstants.Field.SIZE_IN;
        }
        return BioBuzzConstants.Field.CENTER_IN + (k - count() / 2.0) * BioBuzzConstants.Field.TILE_PITCH_IN;
    }

    /** The column (0 = A) that holds this x, clamped to the field. */
    public static int columnOf(double x) {
        return indexOf(x);
    }

    /** The row (0 = row 1) that holds this y, clamped to the field. */
    public static int rowOf(double y) {
        return indexOf(y);
    }

    private static int indexOf(double v) {
        int index = 0;
        while (index < count() - 1 && v >= seamIn(index + 1)) {
            index++;
        }
        return index;
    }

    /** The name of the tile that holds the point, for example {@code "C1"}. */
    public static String nameAt(double x, double y) {
        return name(columnOf(x), rowOf(y));
    }

    /** The name of the tile in column {@code col} (0 = A) and row {@code row} (0 = row 1). */
    public static String name(int col, int row) {
        return "" + (char) ('A' + col) + (row + 1);
    }

    /** The tile with this name (for example {@code "B3"}) as a box, from its seams. Throws IllegalArgumentException for a bad name. */
    public static Rect tile(String name) {
        if (name == null || name.length() != 2) {
            throw new IllegalArgumentException("a tile name is a letter and a digit, like C1, not '" + name + "'");
        }
        int col = name.charAt(0) - 'A';
        int row = name.charAt(1) - '1';
        if (col < 0 || col >= count() || row < 0 || row >= count()) {
            throw new IllegalArgumentException("no tile named '" + name + "'");
        }
        return tile(col, row);
    }

    /** The tile in column {@code col} (0 = A) and row {@code row} (0 = row 1) as a box. */
    public static Rect tile(int col, int row) {
        return new Rect(seamIn(col), seamIn(col + 1), seamIn(row), seamIn(row + 1));
    }

    /** The middle of the tile in column {@code col} (0 = A), along x, in inches. */
    public static double centerX(int col) {
        return (seamIn(col) + seamIn(col + 1)) / 2.0;
    }

    /** The middle of the tile in row {@code row} (0 = row 1), along y, in inches. */
    public static double centerY(int row) {
        return (seamIn(row) + seamIn(row + 1)) / 2.0;
    }
}
