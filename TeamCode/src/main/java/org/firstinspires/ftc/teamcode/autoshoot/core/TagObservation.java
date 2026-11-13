package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * One AprilTag seen in one camera frame, already converted to inches/degrees and to the robot frame.
 * Built by the Limelight adapter; the logic never touches Limelight classes.
 *
 * <p>An AprilTag is a small black-and-white square pattern with an ID number printed into it, which the
 * Limelight can find in its image and measure. On the BIOBUZZ field each HIVE cell carries four of them (IDs
 * 30 to 45 in all); see {@code BioBuzzConstants.Field}. Where this sits in the data flow: {@code LimelightVision}
 * creates these, {@link VisionFrame} carries a list of them, and {@link ClusterAnalyzer} groups them by cell.
 *
 * <p>Every tag always has a direction ({@link #txDeg}, {@link #tyDeg}, {@link #azimuthRad}). It has a 3D
 * position ({@link #forwardIn} and friends) only if the Limelight pipeline's "Full 3D" is on and reported
 * one; a tag without one is "bearing only" and {@link #hasPosition} is false. (A tag whose measured 3D distance
 * is outside {@code TAG_MIN_RANGE_IN}..{@code TAG_MAX_RANGE_IN} is dropped by {@code LimelightVision} and never
 * becomes a TagObservation at all.)
 */
public final class TagObservation {
    /** The AprilTag's ID number (the 36h11 family). HIVE tags are 30 to 45. */
    public final int id;
    /** Degrees; positive means the tag is to the RIGHT of the image centre (conventions already normalized). */
    public final double txDeg;
    /** Degrees; positive means the tag is ABOVE the image centre. */
    public final double tyDeg;
    /** How much of the camera image the tag covers, as the Limelight reports it (percent; bigger = closer). */
    public final double areaPct;
    /** Direction to the tag in the robot's floor plan, radians counter-clockwise from forward (always valid). */
    public final double azimuthRad;
    /** True if the 3D position below is valid. */
    public final boolean hasPosition;
    /** Tag centre in the robot frame, inches: forward, left, and height above the floor. */
    public final double forwardIn;
    /** Inches to the left of the robot's center line (negative = right). NaN if {@link #hasPosition} is false. */
    public final double leftIn;
    /** Height of the tag centre above the floor, inches. NaN if {@link #hasPosition} is false. */
    public final double heightIn;

    private TagObservation(int id, double txDeg, double tyDeg, double areaPct, double azimuthRad,
                           boolean hasPosition, double forwardIn, double leftIn, double heightIn) {
        this.id = id;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.areaPct = areaPct;
        this.azimuthRad = azimuthRad;
        this.hasPosition = hasPosition;
        this.forwardIn = forwardIn;
        this.leftIn = leftIn;
        this.heightIn = heightIn;
    }

    /** A tag with a full 3D position (forward/left/height: robot-frame inches, from the rotation center). */
    public static TagObservation withPosition(int id, double txDeg, double tyDeg, double areaPct,
                                              double azimuthRad, double forwardIn, double leftIn, double heightIn) {
        return new TagObservation(id, txDeg, tyDeg, areaPct, azimuthRad, true, forwardIn, leftIn, heightIn);
    }

    /** A tag we only know the direction to (no distance). The position fields are NaN so misuse is obvious. */
    public static TagObservation bearingOnly(int id, double txDeg, double tyDeg, double areaPct, double azimuthRad) {
        return new TagObservation(id, txDeg, tyDeg, areaPct, azimuthRad, false,
                Double.NaN, Double.NaN, Double.NaN);
    }

    /** Horizontal distance from the robot's centre to the tag, or NaN if there is no 3D position. */
    public double horizontalRangeIn() {
        return hasPosition ? Math.hypot(forwardIn, leftIn) : Double.NaN;
    }
}
