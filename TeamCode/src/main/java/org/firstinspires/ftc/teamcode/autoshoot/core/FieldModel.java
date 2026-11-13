package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The BIOBUZZ field as the auto-shoot logic needs it: the four HIVE cells (two per alliance), the HIVE
 * frame and FLOWER obstacles, and which cell of each HIVE starts UP.
 *
 * <p>Everything is built from the RED-frame numbers in {@link BioBuzzConstants.Field}; the blue
 * cells are the red cells rotated half a turn about the field center. Pure Java, so unit tests can
 * build a model with different numbers through {@link Params}.
 *
 * <p>Think of it as the robot's map. Nothing here moves: it is built once at start-up and then only answers
 * questions ("where is the aim point of the blue rear cell?", "which cell owns AprilTag 35?", "which boxes must
 * the robot stay out of?"). Anything that needs field facts takes a {@code FieldModel} instead of reading the
 * constants itself: {@link ScoringPlanner}, {@link ClusterAnalyzer}, {@link HiveTracker}, the controller and the
 * Limelight Diagnostic OpMode.
 *
 * <p>Frame and vocabulary. Coordinates are the Pedro field frame, in inches: origin at the corner where the
 * audience wall meets the red wall (the inner face of the walls), +x toward blue, +y away from the audience, so the field
 * spans 0..141.348 on both axes ({@code Field.SIZE_IN}, from the official CAD). A HIVE CELL is one scoring opening; the AUDIENCE cell is at small y, the REAR cell at large y. The
 * FLOWERS are four plates on the perimeter walls. A "keep-out" is a box the robot must not enter, and
 * {@link #hardObstacles} are the keep-outs that never move (the HIVE frame and the FLOWERS).
 */
public final class FieldModel {

    /**
     * All inputs of the model. Defaults come from {@link BioBuzzConstants.Field}. The fields are public so a unit
     * test can change one number (for example the field size) and rebuild the model. All lengths are inches.
     */
    public static final class Params {
        /** Side length of the square field. */
        public double size = BioBuzzConstants.Field.SIZE_IN;
        /** Width of the HIVE frame along x. */
        public double frameWidthX = BioBuzzConstants.Field.HIVE_FRAME_WIDTH_X_IN;
        /** Depth of the HIVE frame along y. */
        public double frameDepthY = BioBuzzConstants.Field.HIVE_FRAME_DEPTH_Y_IN;
        /** x of the RED alliance's HIVE axis (the center line of its cell openings). */
        public double redHiveAxisX = BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN;
        /** How far out from the HIVE's pivot plane (y = field center) the aim point of an UP cell is, along y. */
        public double upOriginOut = BioBuzzConstants.Field.UP_CELL_ORIGIN_OUT_IN;
        /** Height of that aim point above the floor. */
        public double upOriginZ = BioBuzzConstants.Field.UP_CELL_ORIGIN_Z_IN;
        /** How far out from the pivot plane the AprilTag row of an UP cell is, along y. */
        public double tagRowOut = BioBuzzConstants.Field.UP_CELL_TAG_ROW_OUT_IN;
        /** Height of the AprilTag row of an UP cell. */
        public double tagRowZ = BioBuzzConstants.Field.UP_CELL_TAG_ROW_Z_IN;
        /** Height of the AprilTag row of a DOWN cell (how the camera tells UP from DOWN by height). */
        public double downTagRowZ = BioBuzzConstants.Field.DOWN_CELL_TAG_ROW_Z_IN;
        /** How far out from the pivot plane the AprilTag row of a DOWN cell is, along y (it swings in toward the pivot as it tips). */
        public double downTagRowOut = BioBuzzConstants.Field.DOWN_CELL_TAG_ROW_OUT_IN;
        /** Tag centers along the sticker, measured from its centerline, in tag-ID order. */
        public double[] tagXOffsets = BioBuzzConstants.Field.TAG_X_OFFSETS_IN;
        /** First AprilTag ID of each cell's four consecutive IDs (see the names for which cell). */
        public int redRearBaseId = BioBuzzConstants.Field.RED_REAR_BASE_TAG_ID;
        public int redAudienceBaseId = BioBuzzConstants.Field.RED_AUDIENCE_BASE_TAG_ID;
        public int blueAudienceBaseId = BioBuzzConstants.Field.BLUE_AUDIENCE_BASE_TAG_ID;
        public int blueRearBaseId = BioBuzzConstants.Field.BLUE_REAR_BASE_TAG_ID;
        /** Which cell of each alliance's HIVE is UP when the match starts. */
        public Side redUpAtStart = BioBuzzConstants.Field.RED_UP_SIDE_AT_MATCH_START;
        public Side blueUpAtStart = BioBuzzConstants.Field.BLUE_UP_SIDE_AT_MATCH_START;
        /** The {x, y} centers of the four FLOWER plates in the RED frame, in field inches. */
        public double[][] flowerCenters = BioBuzzConstants.Field.FLOWER_CENTERS_IN;
        /** Length of a FLOWER keep-out measured along its wall, in inches (centered on the FLOWER's center line). */
        public double flowerAlongWall = BioBuzzConstants.Field.FLOWER_KEEPOUT_ALONG_WALL_IN;
        /** How far a FLOWER keep-out reaches out from its wall, in inches (the plate's field-side edge). */
        public double flowerDepth = BioBuzzConstants.Field.FLOWER_KEEPOUT_DEPTH_IN;
    }

    private final Params p;
    private final CellSpec[][] cells = new CellSpec[2][2];   // [alliance][side]
    private final Rect fieldRect;
    private final Rect hiveFrame;
    private final List<Rect> flowerKeepOuts;

    /** The model with the real field numbers from {@link BioBuzzConstants.Field}. */
    public FieldModel() {
        this(new Params());
    }

    /** A model with custom numbers (used by unit tests). */
    public FieldModel(Params params) {
        this.p = params;
        double c = p.size / 2.0;

        // The two RED cells are written out directly. Both are centered on the HIVE axis x. Along y, the aim point
        // and the tag row are each a fixed distance out from the field's middle line (y = c): the audience cell
        // at c - out, the rear cell at c + out.
        CellSpec redAudience = new CellSpec(Alliance.RED, Side.AUDIENCE, p.redAudienceBaseId, p.redHiveAxisX,
                p.redHiveAxisX, c - p.upOriginOut, p.upOriginZ, c - p.tagRowOut, p.tagRowZ, c - p.downTagRowOut,
                p.downTagRowZ, p.tagXOffsets);
        CellSpec redRear = new CellSpec(Alliance.RED, Side.REAR, p.redRearBaseId, p.redHiveAxisX,
                p.redHiveAxisX, c + p.upOriginOut, p.upOriginZ, c + p.tagRowOut, p.tagRowZ, c + p.downTagRowOut,
                p.downTagRowZ, p.tagXOffsets);
        // Blue cell on side S is the half-turn image of the red cell on the OPPOSITE side. (A half turn about the
        // field center swaps the audience end and the rear end, so the red REAR cell lands on the blue AUDIENCE end.)
        CellSpec blueAudience = rotated(redRear, Alliance.BLUE, Side.AUDIENCE, p.blueAudienceBaseId);
        CellSpec blueRear = rotated(redAudience, Alliance.BLUE, Side.REAR, p.blueRearBaseId);

        cells[Alliance.RED.ordinal()][Side.AUDIENCE.ordinal()] = redAudience;
        cells[Alliance.RED.ordinal()][Side.REAR.ordinal()] = redRear;
        cells[Alliance.BLUE.ordinal()][Side.AUDIENCE.ordinal()] = blueAudience;
        cells[Alliance.BLUE.ordinal()][Side.REAR.ordinal()] = blueRear;

        // The Rect constructor takes (x0, x1, y0, y1): here the whole field, 0..size on both axes.
        fieldRect = new Rect(0, p.size, 0, p.size);
        hiveFrame = Rect.centered(c, c, p.frameWidthX, p.frameDepthY);

        List<Rect> flowers = new ArrayList<Rect>();
        for (double[] f : p.flowerCenters) {
            flowers.add(flowerRect(f[0], f[1]));
        }
        flowerKeepOuts = Collections.unmodifiableList(flowers);
    }

    /**
     * Builds the BLUE cell that is the half-turn image of a RED cell: every x and y becomes (size - value),
     * heights stay the same, and the tag IDs are the blue ones. The facing direction and the tag order are not
     * copied; {@link CellSpec} derives them from the new side.
     */
    private CellSpec rotated(CellSpec red, Alliance alliance, Side side, int baseTagId) {
        double s = p.size;
        return new CellSpec(alliance, side, baseTagId,
                alliance.fromRedX(red.axisX, s),
                alliance.fromRedX(red.originX, s), alliance.fromRedY(red.originY, s), red.originZ,
                alliance.fromRedY(red.tagRowY, s), red.tagRowZ, alliance.fromRedY(red.downTagRowY, s), red.downTagRowZ,
                p.tagXOffsets);
    }

    /**
     * A FLOWER keep-out: a box that starts at the wall the FLOWER stands on and reaches {@code flowerDepth} out from it, with
     * its long side along that wall, centered on the FLOWER's center line. (The FLOWER's center is the middle of its POLLEN stack,
     * a little off the wall, so the box is built from the wall, not around that point: the plate really starts at the wall.)
     */
    private Rect flowerRect(double cx, double cy) {
        double half = p.flowerAlongWall / 2.0;
        // Distance from the center to each of the four walls: the nearest one is the wall the FLOWER is mounted on.
        double toRed = cx;
        double toBlue = p.size - cx;
        double toAudience = cy;
        double toRear = p.size - cy;
        double nearest = Math.min(Math.min(toRed, toBlue), Math.min(toAudience, toRear));
        if (nearest == toRed) {
            return new Rect(0, p.flowerDepth, cy - half, cy + half);
        }
        if (nearest == toBlue) {
            return new Rect(p.size - p.flowerDepth, p.size, cy - half, cy + half);
        }
        if (nearest == toAudience) {
            return new Rect(cx - half, cx + half, 0, p.flowerDepth);
        }
        return new Rect(cx - half, cx + half, p.size - p.flowerDepth, p.size);
    }

    /** Side length of the (square) field in inches; the field spans 0..size on both x and y. */
    public double size() {
        return p.size;
    }

    /** The field's middle on each axis, in inches (70.674): the HIVE sits here, and y = center splits the two sides. */
    public double center() {
        return p.size / 2.0;
    }

    /** The cell of the given alliance's HIVE on the given side (never null). */
    public CellSpec cell(Alliance alliance, Side side) {
        return cells[alliance.ordinal()][side.ordinal()];
    }

    /** Which cell of this alliance's HIVE is UP when the match starts. */
    public Side initialUpSide(Alliance alliance) {
        return alliance == Alliance.RED ? p.redUpAtStart : p.blueUpAtStart;
    }

    /** The whole field as a box (0..size on both axes); the drive bounds are this shrunk by the robot's size. */
    public Rect fieldRect() {
        return fieldRect;
    }

    /** The footprint of the HIVE frame (its feet and base rails). */
    public Rect hiveFrame() {
        return hiveFrame;
    }

    /** The four FLOWER keep-out boxes (unmodifiable), in the same order as the flower centers in the constants. */
    public List<Rect> flowerKeepOuts() {
        return flowerKeepOuts;
    }

    /**
     * Every fixed obstacle a drive path must stay out of: the HIVE frame and the four FLOWERS. Returns a new
     * list each call, so the caller may modify it.
     */
    public List<Rect> hardObstacles() {
        List<Rect> all = new ArrayList<Rect>();
        all.add(hiveFrame);
        all.addAll(flowerKeepOuts);
        return all;
    }

    /**
     * The cell an AprilTag ID belongs to, or null if the ID is not a HIVE tag. With the default numbers: IDs 30-33
     * are the red rear cell, 34-37 red audience, 38-41 blue audience and 42-45 blue rear.
     */
    public CellSpec cellForTag(int tagId) {
        for (Alliance a : Alliance.values()) {
            for (Side s : Side.values()) {
                CellSpec cell = cell(a, s);
                if (cell.ownsTag(tagId)) {
                    return cell;
                }
            }
        }
        return null;
    }
}
