package org.firstinspires.ftc.teamcode.autoshoot.core;

/** What the camera says about one HIVE cell's AprilTag cluster in one frame. */
public final class ClusterView {
    /** Whether the cell looks right-side-up (UP, scorable) or upside-down (DOWN) as seen from the robot. */
    public enum Orientation {
        UP,
        DOWN,
        UNKNOWN
    }

    public final CellSpec cell;
    public final int tagCount;
    public final Orientation orientation;

    /** True if bearing/range/implied position below are valid (an UP cluster with 3D positions). */
    public final boolean hasGeometry;
    /** Centre of the tag row in the robot frame: forward and left, inches. */
    public final double centerForwardIn;
    public final double centerLeftIn;
    /** Horizontal range from the ball exit to the cell's aim point, inches (NaN without geometry). */
    public final double rangeIn;
    /** Counter-clockwise turn that would point the shooter at the aim point, radians (NaN without a bearing). */
    public final double aimErrorRad;
    /** Field position of the robot implied by this cluster and the supplied heading (NaN without geometry). */
    public final double impliedX;
    public final double impliedY;
    public final boolean hasBearing;

    ClusterView(CellSpec cell, int tagCount, Orientation orientation, boolean hasGeometry,
                double centerForwardIn, double centerLeftIn, double rangeIn,
                double aimErrorRad, boolean hasBearing, double impliedX, double impliedY) {
        this.cell = cell;
        this.tagCount = tagCount;
        this.orientation = orientation;
        this.hasGeometry = hasGeometry;
        this.centerForwardIn = centerForwardIn;
        this.centerLeftIn = centerLeftIn;
        this.rangeIn = rangeIn;
        this.aimErrorRad = aimErrorRad;
        this.hasBearing = hasBearing;
        this.impliedX = impliedX;
        this.impliedY = impliedY;
    }

    @Override
    public String toString() {
        return cell + " " + orientation + " tags=" + tagCount
                + (hasGeometry ? String.format(" range=%.1f aimErr=%.1fdeg implied=(%.1f,%.1f)", rangeIn,
                Math.toDegrees(aimErrorRad), impliedX, impliedY) : (hasBearing
                ? String.format(" aimErr=%.1fdeg", Math.toDegrees(aimErrorRad)) : ""));
    }
}
