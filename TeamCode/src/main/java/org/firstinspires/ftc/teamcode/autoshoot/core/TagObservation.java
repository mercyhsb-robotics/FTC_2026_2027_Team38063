package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * One AprilTag seen in one camera frame, already converted to inches/degrees and to the robot frame.
 * Built by the Limelight adapter; the logic never touches Limelight classes.
 */
public final class TagObservation {
    public final int id;
    /** Degrees; positive means the tag is to the RIGHT of the image centre (conventions already normalized). */
    public final double txDeg;
    /** Degrees; positive means the tag is ABOVE the image centre. */
    public final double tyDeg;
    public final double areaPct;
    /** Direction to the tag in the robot's floor plan, radians counter-clockwise from forward (always valid). */
    public final double azimuthRad;
    /** True if the 3D position below is valid. */
    public final boolean hasPosition;
    /** Tag centre in the robot frame, inches: forward, left, and height above the floor. */
    public final double forwardIn;
    public final double leftIn;
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

    public static TagObservation withPosition(int id, double txDeg, double tyDeg, double areaPct,
                                              double azimuthRad, double forwardIn, double leftIn, double heightIn) {
        return new TagObservation(id, txDeg, tyDeg, areaPct, azimuthRad, true, forwardIn, leftIn, heightIn);
    }

    public static TagObservation bearingOnly(int id, double txDeg, double tyDeg, double areaPct, double azimuthRad) {
        return new TagObservation(id, txDeg, tyDeg, areaPct, azimuthRad, false,
                Double.NaN, Double.NaN, Double.NaN);
    }

    /** Horizontal distance from the robot's centre to the tag, or NaN if there is no 3D position. */
    public double horizontalRangeIn() {
        return hasPosition ? Math.hypot(forwardIn, leftIn) : Double.NaN;
    }
}
