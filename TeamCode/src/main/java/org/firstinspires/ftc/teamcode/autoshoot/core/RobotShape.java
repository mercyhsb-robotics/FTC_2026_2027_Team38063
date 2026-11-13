package org.firstinspires.ftc.teamcode.autoshoot.core;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Where the robot's outer edge lies around its POSITION: four distances, in inches, from the robot's position to the
 * front edge, the rear edge, the left edge and the right edge of the perimeter (the rectangle that holds every wheel).
 *
 * <p><b>Why four numbers and not "length and width".</b> The robot's position, the point Pedro Pathing and the
 * odometry report, is its <b>center of rotation</b>: the point the robot spins about. On this robot that point is NOT
 * the middle of the 17.80 x 17.80 in outline. The intake sticks out in front of the front axle, so the front edge is
 * 9.77 in from the position and the rear edge only 8.03 in. (Left and right are 8.90 in each because the drive is
 * symmetric.) A robot that faces a wall with its front touching it therefore has its position 9.77 in from that
 * wall, but facing away with its back on the wall, 8.03 in. Anything that decides "does the robot fit here?" has to use
 * these four numbers, never half the length or half the width.
 *
 * <p>Plain-Java value object, so it runs on the robot and in tests. {@link #fromConstants()} gives the real robot (the
 * {@code FRONT_EXTENT_IN} family in {@link BioBuzzConstants.Robot}); {@link #centered} builds a symmetric rectangle for
 * tests of the generic maths.
 *
 * <p>Vocabulary: <b>forward</b> is the way the intake faces, <b>left</b> is 90 degrees counter-clockwise from it (seen
 * from above). <b>Circumscribed radius</b> (here {@link #reachIn()}): the radius of the smallest circle around the
 * position that holds the whole robot, whichever way it faces.
 */
public final class RobotShape {
    private static final RobotShape REAL = new RobotShape(BioBuzzConstants.Robot.FRONT_EXTENT_IN,
            BioBuzzConstants.Robot.REAR_EXTENT_IN, BioBuzzConstants.Robot.LEFT_EXTENT_IN,
            BioBuzzConstants.Robot.RIGHT_EXTENT_IN);

    private final double frontIn;
    private final double rearIn;
    private final double leftIn;
    private final double rightIn;

    /**
     * @param frontIn distance from the position to the front edge, inches (greater than 0)
     * @param rearIn  distance from the position to the rear edge, inches (greater than 0)
     * @param leftIn  distance from the position to the left edge, inches (greater than 0)
     * @param rightIn distance from the position to the right edge, inches (greater than 0)
     */
    public RobotShape(double frontIn, double rearIn, double leftIn, double rightIn) {
        if (!(frontIn > 0.0) || !(rearIn > 0.0) || !(leftIn > 0.0) || !(rightIn > 0.0)
                || Double.isInfinite(frontIn) || Double.isInfinite(rearIn) || Double.isInfinite(leftIn) || Double.isInfinite(rightIn)) {
            throw new IllegalArgumentException("every distance from the robot's position to its edge must be a positive number of inches: front "
                    + frontIn + ", rear " + rearIn + ", left " + leftIn + ", right " + rightIn);
        }
        this.frontIn = frontIn;
        this.rearIn = rearIn;
        this.leftIn = leftIn;
        this.rightIn = rightIn;
    }

    /** The real robot: the four extents of {@link BioBuzzConstants.Robot}. */
    public static RobotShape fromConstants() {
        return REAL;
    }

    /** A rectangle centered on the position (half the length and half the width each way). For tests of the generic maths only. */
    public static RobotShape centered(double lengthIn, double widthIn) {
        return new RobotShape(lengthIn / 2.0, lengthIn / 2.0, widthIn / 2.0, widthIn / 2.0);
    }

    /** Distance from the position to the front edge, inches. */
    public double frontIn() {
        return frontIn;
    }

    /** Distance from the position to the rear edge, inches. */
    public double rearIn() {
        return rearIn;
    }

    /** Distance from the position to the left edge, inches. */
    public double leftIn() {
        return leftIn;
    }

    /** Distance from the position to the right edge, inches. */
    public double rightIn() {
        return rightIn;
    }

    /** Front to back size of the perimeter, inches (front plus rear). */
    public double lengthIn() {
        return frontIn + rearIn;
    }

    /** Side to side size of the perimeter, inches (left plus right). */
    public double widthIn() {
        return leftIn + rightIn;
    }

    /**
     * The radius of the smallest circle around the position that holds the whole robot, inches: the distance to the
     * farthest corner, {@code hypot(max(front, rear), max(left, right))} (13.22 in for the real robot, not the 12.59 in of a
     * centered 17.80 in square). Spinning in place or turning on the move sweeps the robot out to this radius.
     */
    public double reachIn() {
        return Math.hypot(Math.max(frontIn, rearIn), Math.max(leftIn, rightIn));
    }

    /**
     * The smallest of the four distances, inches (8.03 in for the real robot: the rear). It is how close to a wall the
     * POSITION can possibly be when the robot stands flush against it, whichever way it faces.
     */
    public double smallestExtentIn() {
        return Math.min(Math.min(frontIn, rearIn), Math.min(leftIn, rightIn));
    }

    /**
     * How far the perimeter reaches from the position in a field direction while the robot faces {@code headingRad}: the
     * distance the position must keep from a wall (or any flat edge) that lies in that direction so that the robot only
     * touches it. Example: facing the audience wall front-first (heading -90 degrees) the audience wall is in the
     * direction -90 degrees, so the answer is the front extent, 9.77 in; facing away from it, the rear extent, 8.03 in;
     * sideways, a side extent, 8.90 in; at 45 degrees, a corner, so more than any of them.
     *
     * @param headingRad   the way the robot faces (0 = +x, counter-clockwise positive)
     * @param directionRad the field direction to look in (for example 0 for the wall at the +x end of the field)
     */
    public double reachTowardIn(double headingRad, double directionRad) {
        double a = directionRad - headingRad;      // the direction as seen from the robot's own frame
        double forward = Math.cos(a);
        double left = Math.sin(a);
        // the farthest corner in that direction: the front or the rear edge for the forward part, the left or right for the side part
        return Math.max(frontIn * forward, -rearIn * forward) + Math.max(leftIn * left, -rightIn * left);
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof RobotShape)) {
            return false;
        }
        RobotShape s = (RobotShape) o;
        return frontIn == s.frontIn && rearIn == s.rearIn && leftIn == s.leftIn && rightIn == s.rightIn;
    }

    @Override
    public int hashCode() {
        return java.util.Arrays.hashCode(new double[] {frontIn, rearIn, leftIn, rightIn});
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.ROOT, "robot edges from its position: front %.2f in, rear %.2f in, left %.2f in, right %.2f in",
                frontIn, rearIn, leftIn, rightIn);
    }
}
