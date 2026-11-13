package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * What the camera says about one HIVE cell's AprilTag cluster in one frame.
 *
 * <p>A "cluster" is the group of (up to) four AprilTags stuck on one cell. Instead of reasoning about four
 * separate tags, the logic treats them as one object and asks the cluster: is my cell UP, how far away is the
 * aim point, which way must I turn, and where does that put me on the field?
 *
 * <p>Data flow: {@link ClusterAnalyzer} builds one of these per cell it can see (the constructor is
 * package-private, so only classes in this package can make one); {@link HiveTracker}, {@link PoseCorrector}
 * and {@code AutoShootController} read them.
 *
 * <p>Several fields are only meaningful when {@link #hasGeometry} or {@link #hasBearing} is true; otherwise
 * they are NaN ("not a number"), so a mistake shows up as NaN rather than as a plausible-looking wrong number.
 */
public final class ClusterView {
    /** Whether the cell looks right-side-up (UP, scorable) or upside-down (DOWN) as seen from the robot. */
    public enum Orientation {
        /** This cell is tipped up toward the robot: its opening can be shot into. */
        UP,
        /** This cell is tipped down: do not shoot at it (the OTHER cell of the HIVE is the up one). */
        DOWN,
        /** The tags did not give a clear answer (too few, too close together in the image, or conflicting clues). */
        UNKNOWN
    }

    /** Which cell (alliance, side, tag IDs, field position) these tags belong to. */
    public final CellSpec cell;
    /** How many of the cell's four tags were seen in this frame (1 to 4). */
    public final int tagCount;
    /** The UP/DOWN verdict for this cell from this one frame. */
    public final Orientation orientation;

    /** True if bearing/range/implied position below are valid (an UP cluster with 3D positions). */
    public final boolean hasGeometry;
    /** Centre of the tag row in the robot frame, inches: forward (+ is where the intake faces) and left (+ is left). */
    public final double centerForwardIn;
    public final double centerLeftIn;
    /** Horizontal range from the ball exit to the cell's aim point, inches (NaN without geometry). */
    public final double rangeIn;
    /**
     * Counter-clockwise turn that would point the shooter at the aim point, radians (NaN without a bearing).
     * Positive = turn counter-clockwise (left), negative = clockwise. About 0 means the shooter already points at it.
     */
    public final double aimErrorRad;
    /**
     * Field position of the robot implied by this cluster and the supplied heading (NaN without geometry).
     * "Implied" means: if the tags really are where the field model says, the robot must be standing here.
     * Field frame, inches; the point is the robot's rotation center. Compare with odometry to see drift.
     */
    public final double impliedX;
    public final double impliedY;
    /** True if {@link #aimErrorRad} is valid. Needs an UP verdict but, unlike geometry, not 3D positions. */
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

    // A one-line summary for the driver's telemetry screen (aim error in degrees, range and position in inches).
    @Override
    public String toString() {
        return cell + " " + orientation + " tags=" + tagCount
                + (hasGeometry ? String.format(" range=%.1f aimErr=%.1fdeg implied=(%.1f,%.1f)", rangeIn,
                Math.toDegrees(aimErrorRad), impliedX, impliedY) : (hasBearing
                ? String.format(" aimErr=%.1fdeg", Math.toDegrees(aimErrorRad)) : ""));
    }
}
