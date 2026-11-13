package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Where one HIVE CELL is, and which AprilTags belong to it, for the case that the cell is UP.
 *
 * <p>The numbers describe the cell's UP pose. When a HIVE tips, the cell on the other side becomes
 * the UP one; the geometry of a cell that is currently DOWN is not needed because it cannot be shot
 * into.
 */
public final class CellSpec {
    public final Alliance alliance;
    public final Side side;

    /** First of the four consecutive AprilTag IDs on this cell's sticker. */
    public final int baseTagId;

    /** x of the HIVE axis this cell hangs from (the center line of its 20 in wide opening). */
    public final double axisX;

    /** Field position of the aim point (center of the opening) when this cell is UP, in inches. */
    public final double originX;
    public final double originY;
    public final double originZ;

    /** Field y and height of the AprilTag row when this cell is UP. */
    public final double tagRowY;
    public final double tagRowZ;

    /** +1 if increasing tag ID runs toward +x on the field, -1 if toward -x (rear-side clusters are rotated). */
    public final int tagXSign;

    /** Direction the opening faces in the floor plan (radians): -y for the audience side, +y for the rear. */
    public final double facingHeading;

    private final double[] tagXOffsets;

    public CellSpec(Alliance alliance, Side side, int baseTagId, double axisX,
                    double originX, double originY, double originZ,
                    double tagRowY, double tagRowZ, double[] tagXOffsets) {
        this.alliance = alliance;
        this.side = side;
        this.baseTagId = baseTagId;
        this.axisX = axisX;
        this.originX = originX;
        this.originY = originY;
        this.originZ = originZ;
        this.tagRowY = tagRowY;
        this.tagRowZ = tagRowZ;
        this.tagXSign = side == Side.AUDIENCE ? 1 : -1;
        this.facingHeading = side.openingFacingHeading();
        this.tagXOffsets = tagXOffsets.clone();
    }

    public int tagCount() {
        return tagXOffsets.length;
    }

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
