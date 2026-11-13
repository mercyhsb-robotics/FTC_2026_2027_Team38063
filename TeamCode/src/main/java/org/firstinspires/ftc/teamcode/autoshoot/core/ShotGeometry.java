package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Geometry between a robot pose and a cell's aim point. The ball leaves the shooter at the LAUNCH
 * POINT (a fixed spot on the robot, behind its center because the shooter fires out the back), and
 * the shooter fires along the robot's backward direction, so the heading that points it at a target
 * is the bearing to the target plus half a turn.
 *
 * <p>All distances are horizontal, in inches, measured in the floor plan.
 */
public final class ShotGeometry {
    private ShotGeometry() {
    }

    /** Field x of the ball exit for a robot pose. */
    public static double launchX(Pose2 robot, double launchForwardIn, double launchLeftIn) {
        return robot.fieldX(launchForwardIn, launchLeftIn);
    }

    /** Field y of the ball exit for a robot pose. */
    public static double launchY(Pose2 robot, double launchForwardIn, double launchLeftIn) {
        return robot.fieldY(launchForwardIn, launchLeftIn);
    }

    /** Horizontal distance from the ball exit to a point. */
    public static double rangeTo(Pose2 robot, double launchForwardIn, double launchLeftIn,
                                 double targetX, double targetY) {
        return Math.hypot(targetX - launchX(robot, launchForwardIn, launchLeftIn),
                targetY - launchY(robot, launchForwardIn, launchLeftIn));
    }

    /**
     * The heading at which the shooter (which fires out the back) points straight at the target,
     * assuming the ball exit stays where it is.
     */
    public static double aimHeading(Pose2 robot, double launchForwardIn, double launchLeftIn,
                                    double targetX, double targetY) {
        double lx = launchX(robot, launchForwardIn, launchLeftIn);
        double ly = launchY(robot, launchForwardIn, launchLeftIn);
        return Angles.opposite(Math.atan2(targetY - ly, targetX - lx));
    }

    /** Signed turn needed (counter-clockwise positive) to point the shooter at the target. */
    public static double aimError(Pose2 robot, double launchForwardIn, double launchLeftIn,
                                  double targetX, double targetY) {
        return Angles.error(robot.heading, aimHeading(robot, launchForwardIn, launchLeftIn, targetX, targetY));
    }

    /**
     * How far off the opening's centre axis the shooter stands, in radians (signed). Zero means the
     * ball exit is exactly out in front of the opening along the direction the opening faces.
     */
    public static double azimuthOffAxis(Pose2 robot, double launchForwardIn, double launchLeftIn, CellSpec cell) {
        double lx = launchX(robot, launchForwardIn, launchLeftIn);
        double ly = launchY(robot, launchForwardIn, launchLeftIn);
        double fromAim = Math.atan2(ly - cell.originY, lx - cell.originX);
        return Angles.error(cell.facingHeading, fromAim);
    }

    /** The pose whose ball exit is at (launchX, launchY) with the shooter pointed at (aimX, aimY). */
    public static Pose2 poseForLaunchPoint(double launchX, double launchY, double aimX, double aimY,
                                           double launchForwardIn, double launchLeftIn) {
        double heading = Angles.opposite(Math.atan2(aimY - launchY, aimX - launchX));
        double cx = launchX - (launchForwardIn * Math.cos(heading) - launchLeftIn * Math.sin(heading));
        double cy = launchY - (launchForwardIn * Math.sin(heading) + launchLeftIn * Math.cos(heading));
        return new Pose2(cx, cy, heading);
    }

    /** Width of the axis-aligned box around a robot footprint of the given size turned by {@code heading}. */
    public static double footprintExtentX(double lengthIn, double widthIn, double heading) {
        return lengthIn * Math.abs(Math.cos(heading)) + widthIn * Math.abs(Math.sin(heading));
    }

    /** Height of the axis-aligned box around a rotated robot footprint. */
    public static double footprintExtentY(double lengthIn, double widthIn, double heading) {
        return lengthIn * Math.abs(Math.sin(heading)) + widthIn * Math.abs(Math.cos(heading));
    }

    /** The axis-aligned box around the robot footprint at a pose. */
    public static Rect footprint(Pose2 pose, double lengthIn, double widthIn) {
        return Rect.centered(pose.x, pose.y,
                footprintExtentX(lengthIn, widthIn, pose.heading),
                footprintExtentY(lengthIn, widthIn, pose.heading));
    }
}
