package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Geometry between a robot pose and a cell's aim point. The ball leaves the shooter at the LAUNCH
 * POINT (a fixed spot on the robot, behind its center because the shooter fires out the back), and
 * the shooter fires along the robot's backward direction, so the heading that points it at a target
 * is the bearing to the target plus half a turn.
 *
 * <p>All distances are horizontal, in inches, measured in the floor plan.
 *
 * <p>Conventions used by every method here:
 * <ul>
 *   <li>A {@link Pose2} is where the robot's rotation center is on the field (Pedro field frame, inches) and which
 *       way it faces (radians, 0 = +x, counter-clockwise positive).</li>
 *   <li>{@code launchForwardIn} / {@code launchLeftIn} locate the ball exit relative to that rotation center, in
 *       the robot's own frame: forward is the way the intake faces, left is 90 degrees counter-clockwise from it.
 *       The shooter is at the back, so {@code launchForwardIn} is negative (about -8.03 in: the rear edge of the robot,
 *       which is NOT the same distance behind the center of rotation as the front edge is in front of it).</li>
 *   <li>{@code targetX}, {@code targetY} are field inches; normally a cell's aim point
 *       ({@link CellSpec#originX}, {@link CellSpec#originY}).</li>
 * </ul>
 *
 * <p>Callers: {@link AutoShootController} (current range and aim error while driving and shooting) and
 * {@link ScoringPlanner} (which pose puts the ball exit at a chosen spot, and whether the robot fits there).
 * The ball exit sits off-center, so turning the robot also swings the exit around; that is why the "which
 * heading" and "which pose" questions are answered here once, in one place.
 */
public final class ShotGeometry {
    private ShotGeometry() {
    }

    /** Field x of the ball exit for a robot pose, in inches. */
    public static double launchX(Pose2 robot, double launchForwardIn, double launchLeftIn) {
        return robot.fieldX(launchForwardIn, launchLeftIn);
    }

    /** Field y of the ball exit for a robot pose, in inches. */
    public static double launchY(Pose2 robot, double launchForwardIn, double launchLeftIn) {
        return robot.fieldY(launchForwardIn, launchLeftIn);
    }

    /**
     * Horizontal distance from the ball exit to a point, in inches. This is the "range" the shot table is indexed
     * by (see {@link ShotTable}).
     */
    public static double rangeTo(Pose2 robot, double launchForwardIn, double launchLeftIn,
                                 double targetX, double targetY) {
        return Math.hypot(targetX - launchX(robot, launchForwardIn, launchLeftIn),
                targetY - launchY(robot, launchForwardIn, launchLeftIn));
    }

    /**
     * The heading at which the shooter (which fires out the back) points straight at the target,
     * assuming the ball exit stays where it is. In radians, in (-PI, PI]. (Heading + 180 degrees because the
     * direction TO the target must be the robot's BACK, not its front.)
     */
    public static double aimHeading(Pose2 robot, double launchForwardIn, double launchLeftIn,
                                    double targetX, double targetY) {
        double lx = launchX(robot, launchForwardIn, launchLeftIn);
        double ly = launchY(robot, launchForwardIn, launchLeftIn);
        return Angles.opposite(Math.atan2(targetY - ly, targetX - lx));
    }

    /**
     * Signed turn needed (counter-clockwise positive) to point the shooter at the target, in radians. Zero means
     * the shooter already points at it. It is the shortest way around, so it never exceeds half a turn.
     */
    public static double aimError(Pose2 robot, double launchForwardIn, double launchLeftIn,
                                  double targetX, double targetY) {
        return Angles.error(robot.heading, aimHeading(robot, launchForwardIn, launchLeftIn, targetX, targetY));
    }

    /**
     * How far off the opening's centre axis the shooter stands, in radians (signed). Zero means the
     * ball exit is exactly out in front of the opening along the direction the opening faces.
     * Positive means the exit is counter-clockwise (seen from above) of that axis around the aim point; for an
     * audience-side cell, that is toward +x. The planner limits this to the "shoot cone" half-angle.
     */
    public static double azimuthOffAxis(Pose2 robot, double launchForwardIn, double launchLeftIn, CellSpec cell) {
        double lx = launchX(robot, launchForwardIn, launchLeftIn);
        double ly = launchY(robot, launchForwardIn, launchLeftIn);
        double fromAim = Math.atan2(ly - cell.originY, lx - cell.originX);
        return Angles.error(cell.facingHeading, fromAim);
    }

    /**
     * The pose whose ball exit is at (launchX, launchY) with the shooter pointed at (aimX, aimY). This is the
     * planner's "inverse" step: it first decides where the ball exit should be, then asks where the robot's
     * rotation center must therefore stand. Returns a pose in field inches / radians.
     */
    public static Pose2 poseForLaunchPoint(double launchX, double launchY, double aimX, double aimY,
                                           double launchForwardIn, double launchLeftIn) {
        double heading = Angles.opposite(Math.atan2(aimY - launchY, aimX - launchX));
        // The exit is (launchForwardIn, launchLeftIn) from the center, turned by the heading; walk back from the exit.
        double cx = launchX - (launchForwardIn * Math.cos(heading) - launchLeftIn * Math.sin(heading));
        double cy = launchY - (launchForwardIn * Math.sin(heading) + launchLeftIn * Math.cos(heading));
        return new Pose2(cx, cy, heading);
    }

    /**
     * The axis-aligned box ({@link Rect}) around the robot's perimeter at a pose. The perimeter is NOT centered on the
     * pose (the robot's center of rotation): it reaches {@link RobotShape#frontIn()} in front of it, {@link
     * RobotShape#rearIn()} behind it and so on, turned to the heading, so the box is not centered on the pose either.
     * The planner uses it to ask "is the robot touching a wall or an obstacle here?". Example: the real robot facing the
     * audience wall (heading -90 degrees) at y = 9.77 has a box that starts at y = 0.
     */
    public static Rect footprint(Pose2 pose, RobotShape robot) {
        return Footprint.of(pose, robot).bounds();
    }
}
