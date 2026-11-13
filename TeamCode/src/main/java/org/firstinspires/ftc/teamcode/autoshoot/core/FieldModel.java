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
 */
public final class FieldModel {

    /** All inputs of the model. Defaults come from {@link BioBuzzConstants.Field}. */
    public static final class Params {
        public double size = BioBuzzConstants.Field.SIZE_IN;
        public double frameWidthX = BioBuzzConstants.Field.HIVE_FRAME_WIDTH_X_IN;
        public double frameDepthY = BioBuzzConstants.Field.HIVE_FRAME_DEPTH_Y_IN;
        public double redHiveAxisX = BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN;
        public double upOriginOut = BioBuzzConstants.Field.UP_CELL_ORIGIN_OUT_IN;
        public double upOriginZ = BioBuzzConstants.Field.UP_CELL_ORIGIN_Z_IN;
        public double tagRowOut = BioBuzzConstants.Field.UP_CELL_TAG_ROW_OUT_IN;
        public double tagRowZ = BioBuzzConstants.Field.UP_CELL_TAG_ROW_Z_IN;
        public double downTagRowZ = BioBuzzConstants.Field.DOWN_CELL_TAG_ROW_Z_IN;
        public double[] tagXOffsets = BioBuzzConstants.Field.TAG_X_OFFSETS_IN;
        public int redRearBaseId = BioBuzzConstants.Field.RED_REAR_BASE_TAG_ID;
        public int redAudienceBaseId = BioBuzzConstants.Field.RED_AUDIENCE_BASE_TAG_ID;
        public int blueAudienceBaseId = BioBuzzConstants.Field.BLUE_AUDIENCE_BASE_TAG_ID;
        public int blueRearBaseId = BioBuzzConstants.Field.BLUE_REAR_BASE_TAG_ID;
        public Side redUpAtStart = BioBuzzConstants.Field.RED_UP_SIDE_AT_MATCH_START;
        public Side blueUpAtStart = BioBuzzConstants.Field.BLUE_UP_SIDE_AT_MATCH_START;
        public double[][] flowerCenters = BioBuzzConstants.Field.FLOWER_CENTERS_IN;
        public double flowerAlongWall = BioBuzzConstants.Field.FLOWER_KEEPOUT_ALONG_WALL_IN;
        public double flowerDepth = BioBuzzConstants.Field.FLOWER_KEEPOUT_DEPTH_IN;
    }

    private final Params p;
    private final CellSpec[][] cells = new CellSpec[2][2];   // [alliance][side]
    private final Rect fieldRect;
    private final Rect hiveFrame;
    private final List<Rect> flowerKeepOuts;

    public FieldModel() {
        this(new Params());
    }

    public FieldModel(Params params) {
        this.p = params;
        double c = p.size / 2.0;

        CellSpec redAudience = new CellSpec(Alliance.RED, Side.AUDIENCE, p.redAudienceBaseId, p.redHiveAxisX,
                p.redHiveAxisX, c - p.upOriginOut, p.upOriginZ, c - p.tagRowOut, p.tagRowZ, p.downTagRowZ,
                p.tagXOffsets);
        CellSpec redRear = new CellSpec(Alliance.RED, Side.REAR, p.redRearBaseId, p.redHiveAxisX,
                p.redHiveAxisX, c + p.upOriginOut, p.upOriginZ, c + p.tagRowOut, p.tagRowZ, p.downTagRowZ,
                p.tagXOffsets);
        // Blue cell on side S is the half-turn image of the red cell on the OPPOSITE side.
        CellSpec blueAudience = rotated(redRear, Alliance.BLUE, Side.AUDIENCE, p.blueAudienceBaseId);
        CellSpec blueRear = rotated(redAudience, Alliance.BLUE, Side.REAR, p.blueRearBaseId);

        cells[Alliance.RED.ordinal()][Side.AUDIENCE.ordinal()] = redAudience;
        cells[Alliance.RED.ordinal()][Side.REAR.ordinal()] = redRear;
        cells[Alliance.BLUE.ordinal()][Side.AUDIENCE.ordinal()] = blueAudience;
        cells[Alliance.BLUE.ordinal()][Side.REAR.ordinal()] = blueRear;

        fieldRect = new Rect(0, p.size, 0, p.size);
        hiveFrame = Rect.centered(c, c, p.frameWidthX, p.frameDepthY);

        List<Rect> flowers = new ArrayList<Rect>();
        for (double[] f : p.flowerCenters) {
            flowers.add(flowerRect(f[0], f[1]));
        }
        flowerKeepOuts = Collections.unmodifiableList(flowers);
    }

    private CellSpec rotated(CellSpec red, Alliance alliance, Side side, int baseTagId) {
        double s = p.size;
        return new CellSpec(alliance, side, baseTagId,
                alliance.fromRedX(red.axisX, s),
                alliance.fromRedX(red.originX, s), alliance.fromRedY(red.originY, s), red.originZ,
                alliance.fromRedY(red.tagRowY, s), red.tagRowZ, red.downTagRowZ, p.tagXOffsets);
    }

    /** A FLOWER keep-out, oriented so its long side runs along whichever perimeter wall it sits on. */
    private Rect flowerRect(double cx, double cy) {
        double toLeftRight = Math.min(cx, p.size - cx);
        double toBottomTop = Math.min(cy, p.size - cy);
        if (toLeftRight <= toBottomTop) {
            return Rect.centered(cx, cy, p.flowerDepth, p.flowerAlongWall);   // on a left/right wall
        }
        return Rect.centered(cx, cy, p.flowerAlongWall, p.flowerDepth);       // on the audience/rear wall
    }

    public double size() {
        return p.size;
    }

    public double center() {
        return p.size / 2.0;
    }

    public CellSpec cell(Alliance alliance, Side side) {
        return cells[alliance.ordinal()][side.ordinal()];
    }

    /** Which cell of this alliance's HIVE is UP when the match starts. */
    public Side initialUpSide(Alliance alliance) {
        return alliance == Alliance.RED ? p.redUpAtStart : p.blueUpAtStart;
    }

    public Rect fieldRect() {
        return fieldRect;
    }

    /** The footprint of the HIVE frame (its feet and base rails). */
    public Rect hiveFrame() {
        return hiveFrame;
    }

    public List<Rect> flowerKeepOuts() {
        return flowerKeepOuts;
    }

    /** Every fixed obstacle a drive path must stay out of: the HIVE frame and the four FLOWERS. */
    public List<Rect> hardObstacles() {
        List<Rect> all = new ArrayList<Rect>();
        all.add(hiveFrame);
        all.addAll(flowerKeepOuts);
        return all;
    }

    /** The cell an AprilTag ID belongs to, or null if the ID is not a HIVE tag. */
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
