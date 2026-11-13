package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Where one HIVE CELL is, and which AprilTags belong to it, for the case that the cell is UP.
 *
 * <p>Vocabulary: a HIVE is the goal. Each alliance's HIVE has two CELLS, one on the audience side and one on the
 * rear side (see {@link Side}). Exactly one cell is tipped UP, which is the only one POLLEN may be shot into. Each
 * cell carries a sticker with four consecutive AprilTags (a "cluster"). The middle of the cell's opening is the
 * AIM POINT; the code also calls it the cluster "origin".
 *
 * <p>The numbers describe the cell's UP pose. When a HIVE tips, the cell on the other side becomes
 * the UP one; the geometry of a cell that is currently DOWN is not needed because it cannot be shot
 * into.
 *
 * <p>This is a plain data holder (four exist, one per alliance and side). {@link FieldModel} builds them from
 * {@code BioBuzzConstants.Field}. {@link ScoringPlanner} reads the aim point and facing direction to choose where
 * to stand, {@link ClusterAnalyzer} reads the tag layout to turn camera readings into a range and a bearing,
 * and {@link AutoShootController} aims at the origin.
 *
 * <p>Frame: Pedro field frame, inches (origin at the inner corner where the audience wall meets the red wall, +x toward
 * blue, +y away from the audience). "z" is height above the floor.
 */
public final class CellSpec {
    public final Alliance alliance;
    public final Side side;

    /** First of the four consecutive AprilTag IDs on this cell's sticker. */
    public final int baseTagId;

    /** x of the HIVE axis this cell hangs from (the center line of its 20 in wide opening), in field inches. */
    public final double axisX;

    /** Field position of the aim point (center of the opening) when this cell is UP, in inches. */
    public final double originX;
    public final double originY;
    /** Height of the aim point above the floor, in inches. The logic works in the floor plan and does not read it. */
    public final double originZ;

    /** Field y and height of the AprilTag row when this cell is UP, in inches. */
    public final double tagRowY;
    public final double tagRowZ;

    /**
     * Field y and height of the AprilTag row when this cell is DOWN. A tip turns the cell by 60 degrees about the pivot axle, so the
     * DOWN row is the UP row turned about it: 14.3 in lower, and 1.5 in nearer the pivot plane (the sticker plane is the same plane).
     */
    public final double downTagRowY;
    public final double downTagRowZ;

    /**
     * +1 if increasing tag ID runs toward +x on the field, -1 if toward -x (rear-side clusters are rotated).
     * In practice: +1 for AUDIENCE-side cells and -1 for REAR-side cells, for both alliances.
     */
    public final int tagXSign;

    /** Direction the opening faces in the floor plan (radians): -y for the audience side, +y for the rear. */
    public final double facingHeading;

    /** Tag centers along the sticker, in inches from its centerline, in tag-ID order. */
    private final double[] tagXOffsets;

    /**
     * Normally called only by {@link FieldModel}, which knows the real numbers. The origin is the aim point; the
     * tag row is the line of tag centers. The tag offsets are copied, so the caller's array can change safely.
     */
    public CellSpec(Alliance alliance, Side side, int baseTagId, double axisX,
                    double originX, double originY, double originZ,
                    double tagRowY, double tagRowZ, double downTagRowY, double downTagRowZ, double[] tagXOffsets) {
        this.alliance = alliance;
        this.side = side;
        this.baseTagId = baseTagId;
        this.axisX = axisX;
        this.originX = originX;
        this.originY = originY;
        this.originZ = originZ;
        this.tagRowY = tagRowY;
        this.tagRowZ = tagRowZ;
        this.downTagRowY = downTagRowY;
        this.downTagRowZ = downTagRowZ;
        this.tagXSign = side == Side.AUDIENCE ? 1 : -1;
        this.facingHeading = side.openingFacingHeading();
        this.tagXOffsets = tagXOffsets.clone();
    }

    /** How many tags the sticker has (four). */
    public int tagCount() {
        return tagXOffsets.length;
    }

    /** True if this AprilTag ID is one of this cell's tags. */
    public boolean ownsTag(int id) {
        return id >= baseTagId && id < baseTagId + tagXOffsets.length;
    }

    /** Index 0..3 of a tag within the cluster (its position in ID order), or -1 if it is not ours. */
    public int tagIndex(int id) {
        return ownsTag(id) ? id - baseTagId : -1;
    }

    /** Field x of the center of tag number {@code index} (0-3) when this cell is UP. */
    public double tagFieldX(int index) {
        return axisX + tagXSign * tagXOffsets[index];
    }

    /** Offset of tag {@code index} along the sticker from its centerline, in inches (cluster frame). */
    public double tagOffsetAlongSticker(int index) {
        return tagXOffsets[index];
    }

    @Override
    public String toString() {
        return alliance + " " + side + " cell (tags " + baseTagId + "-" + (baseTagId + tagCount() - 1) + ")";
    }
}
