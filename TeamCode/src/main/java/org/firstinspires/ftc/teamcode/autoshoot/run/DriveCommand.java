package org.firstinspires.ftc.teamcode.autoshoot.run;

import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.DrivePath;

/**
 * What the route runner asks the drive to do this loop. It is a plain description with no Pedro Pathing in it, so the same
 * object can be handed to the real drive ({@code PedroDrive.apply(DriveCommand)}) or to the simulated robot of a unit test.
 *
 * <p>There are three kinds, and each one already exists as a way to drive in the hold-to-shoot feature:
 * <ul>
 *   <li>{@link Kind#HOLD}: stay at one pose and push back if bumped. Used for waiting, for the wiggle at a FLOWER (the position stays
 *       put and only the target heading changes) and for the pose where a route ends.</li>
 *   <li>{@link Kind#WAYPOINTS}: drive through a list of points to a destination, turning once, evenly, by the rule of
 *       {@code HeadingProfile} (the same rule the route checker uses). It is the plain straight move to the pose of a SHOOT, an INTAKE or a PARK,
 *       and the route the shooting planner has found.</li>
 *   <li>{@link Kind#LEG}: follow the {@link DrivePath} of a DRIVE step: a straight line or a Bezier curve, with the heading rule the step names
 *       (tangent, reverse tangent, linear, constant or facing a point). The drive builds the same path the validator swept, so what is
 *       checked is what is driven.</li>
 * </ul>
 *
 * <p>The drive remembers the command by IDENTITY: a route or a leg is only (re)started when the {@code waypoints} list or the {@code leg} object is a
 * different object from the one it is already following, so the runner keeps handing in the same object for as long as the move lasts.
 */
public final class DriveCommand {
    /** The three ways to drive. */
    public enum Kind { HOLD, WAYPOINTS, LEG }

    /** Which way. */
    public final Kind kind;
    /**
     * HOLD: the pose to hold. WAYPOINTS: where the route ends, and the heading to have there. LEG: the pose the leg ends in (for the drive's own
     * arrival checks and for the screen).
     */
    public final Pose2 pose;
    /** WAYPOINTS: the points to drive through, each {@code {x, y}} in inches, the last one is the destination. Empty for the other kinds. */
    public final List<double[]> waypoints;
    /** WAYPOINTS: keep the heading the robot has for this many inches before turning (see {@code HeadingProfile}). */
    public final double turnHoldIn;
    /** WAYPOINTS: be finished turning this many inches before the route ends. */
    public final double turnSettleIn;
    /** LEG: the path to follow (null for the other kinds). */
    public final DrivePath leg;
    /** LEG: the fraction (0, 1] of the cruise speed to drive at; 1 for the other kinds. */
    public final double speedFraction;

    private DriveCommand(Kind kind, Pose2 pose, List<double[]> waypoints, double turnHoldIn, double turnSettleIn, DrivePath leg,
                         double speedFraction) {
        this.kind = kind;
        this.pose = pose;
        this.waypoints = waypoints;
        this.turnHoldIn = turnHoldIn;
        this.turnSettleIn = turnSettleIn;
        this.leg = leg;
        this.speedFraction = speedFraction;
    }

    /** Stay at {@code pose}. */
    public static DriveCommand hold(Pose2 pose) {
        return new DriveCommand(Kind.HOLD, pose, Collections.<double[]>emptyList(), 0.0, 0.0, null, 1.0);
    }

    /** Drive through {@code waypoints} (the list object is kept, not copied) and end at {@code end}, turning evenly. */
    public static DriveCommand waypoints(List<double[]> waypoints, Pose2 end, double turnHoldIn, double turnSettleIn) {
        return new DriveCommand(Kind.WAYPOINTS, end, waypoints, turnHoldIn, turnSettleIn, null, 1.0);
    }

    /** Follow a DRIVE leg at {@code speedFraction} of the cruise speed. */
    public static DriveCommand leg(DrivePath leg, double speedFraction) {
        return new DriveCommand(Kind.LEG, leg.end(), Collections.<double[]>emptyList(), 0.0, 0.0, leg, speedFraction);
    }

    @Override
    public String toString() {
        switch (kind) {
            case WAYPOINTS:
                return "WAYPOINTS(" + waypoints.size() + " points) to " + pose;
            case LEG:
                return "LEG(" + leg.mode() + ", " + String.format(java.util.Locale.ROOT, "%.1f", leg.length()) + " in) to " + pose;
            default:
                return "HOLD " + pose;
        }
    }
}
