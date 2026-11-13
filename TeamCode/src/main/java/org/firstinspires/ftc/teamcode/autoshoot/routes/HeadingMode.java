package org.firstinspires.ftc.teamcode.autoshoot.routes;

/**
 * How the robot's heading is controlled along one DRIVE leg. The names follow Pedro Pathing's
 * heading interpolators so that a leg converts to a Pedro path one to one.
 */
public enum HeadingMode {
    /** The FRONT (intake) of the robot points along the direction of travel. The target heading is ignored. */
    TANGENT,
    /**
     * The BACK (shooter) of the robot points along the direction of travel, so the robot drives
     * backwards, shooter first. The target heading is ignored (Pedro's {@code reverseTangent()}).
     */
    REVERSE_TANGENT,
    /** The heading turns evenly from the heading the leg starts with to the target heading. */
    LINEAR,
    /** The robot turns to the target heading at the start of the leg and holds it all the way. */
    CONSTANT,
    /** The robot keeps facing a fixed field point (front, or back when the leg asks for it). */
    FACE_POINT
}
