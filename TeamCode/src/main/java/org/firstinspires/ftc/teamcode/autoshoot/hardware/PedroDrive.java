package org.firstinspires.ftc.teamcode.autoshoot.hardware;

import java.util.List;

import com.pedropathing.api.Paths;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.AutoShootController;
import org.firstinspires.ftc.teamcode.autoshoot.core.HeadingProfile;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.DrivePath;
import org.firstinspires.ftc.teamcode.autoshoot.run.DriveCommand;

/**
 * The only class that talks to Pedro Pathing for the auto-shoot feature. It turns the controller's
 * drive commands into Follower calls and converts between Pedro's pose (heading in [0, 2*PI)) and the
 * logic's {@link Pose2} (heading in (-PI, PI]).
 *
 * <p>It owns the four drive motors through the Follower, so the TeleOp drives manually through
 * {@link #manual} as well; never run another class that writes drive power at the same time.
 *
 * <p>Pedro facts this relies on: {@code follow(path)} and {@code hold(pose)} both reset the follower's
 * state, so they are only re-issued when the command actually changes; after a path finishes the
 * follower keeps holding its end pose; {@code isBusy()} does not mean "arrived", so arrival is judged
 * by the controller from the measured pose instead.
 *
 * <p>Words used here. <b>Pedro Pathing</b> is the library that drives the robot along paths and holds
 * positions. Its <b>Follower</b> is the object that does the work: every loop its {@code update()} asks the
 * localizer "where am I?", works out how to correct the error, and sets the four motor powers. Our localizer is
 * the goBILDA <b>Pinpoint</b>, an odometry computer that tracks the robot's position from two free-spinning
 * wheels plus a gyro ("odometry" = working out where you are from how far the wheels turned). Pedro works in
 * inches and radians in the same field frame as {@link Pose2}.
 *
 * <p>The three ways the robot can be driven, which {@link #apply} chooses between each loop:
 * <ul>
 *   <li><b>MANUAL</b>: the driver's sticks, no position control;</li>
 *   <li><b>FOLLOW</b>: drive along a list of waypoints (a route around obstacles) to a destination;</li>
 *   <li><b>HOLD</b>: actively stay at one pose, pushing back if bumped.</li>
 * </ul>
 *
 * <p>Data flow: {@code AutoShootController} decides (its {@code Outputs.driveMode}); {@code BioBuzzRobot} calls
 * {@link #update} first thing every loop, reads {@link #pose}, {@link #speedInPerSec} and {@link #omegaRadPerSec}
 * to feed the controller, then calls {@link #apply}.
 */
public final class PedroDrive {
    /**
     * A hold command is re-issued only if the target moved by more than this (inches / radians). Re-issuing
     * resets Pedro's internal state, so doing it every loop for a target that barely changes would make the
     * robot twitch.
     */
    private static final double HOLD_REISSUE_IN = 0.05;
    private static final double HOLD_REISSUE_RAD = Math.toRadians(0.25);

    // Which kind of command we last gave the follower (NONE = nothing yet, or we just reset the pose).
    private enum Applied { NONE, MANUAL, FOLLOW, HOLD, LEG }

    private final Follower follower;
    private Applied applied = Applied.NONE;
    // The waypoint list of the route being followed. Compared by identity (==), see follow().
    private List<double[]> activeRoute;
    // The pose of the last hold command, to tell whether a new hold command really moved the target.
    private Pose2 lastHold;
    // The DRIVE leg being followed (compared by identity, like the waypoint list), see followLeg().
    private DrivePath activeLeg;
    // Pedro's setting that caps the speed along a path (the ForesightConfig's maxPathSpeed), or null if the caller gave none (then a slow leg runs at full speed).
    private ConfigVar<Double> pathSpeedLimit;

    public PedroDrive(Follower follower) {
        this.follower = follower;
    }

    /**
     * Gives the drive the Pedro setting that caps the speed along a path, {@code Constants.foresightConfig.maxPathSpeed}, so that a DRIVE leg
     * (or the slow approach to a FLOWER) that asks for a fraction of the cruise speed really gets it. It is applied to that one path only. Without
     * it a slow leg runs at the normal speed. Returns this drive so it can be chained after the constructor.
     */
    public PedroDrive withPathSpeedLimit(ConfigVar<Double> maxPathSpeed) {
        this.pathSpeedLimit = maxPathSpeed;
        return this;
    }

    /** The underlying Pedro Follower, for code that needs something this wrapper does not offer. */
    public Follower follower() {
        return follower;
    }

    /** Runs the localizer and the motors; call exactly once per loop, first. */
    public void update() {
        follower.update();
    }

    /** Where odometry says the robot is: field frame, inches and radians, heading wrapped into (-PI, PI]. */
    public Pose2 pose() {
        Pose p = follower.pose();
        // The Pose2 constructor wraps the heading, so Pedro's [0, 2*PI) becomes (-PI, PI] here.
        return new Pose2(p.x(), p.y(), p.heading());
    }

    /** Translation speed in inches per second. */
    public double speedInPerSec() {
        return Math.hypot(follower.velocity().vx, follower.velocity().vy);
    }

    /** Turn rate in radians per second (the callers only use its size). */
    public double omegaRadPerSec() {
        return follower.velocity().omega;
    }

    /**
     * Tells odometry where the robot really is (field frame: inches, radians). Used at the start of the OpMode,
     * when the driver says "I am on the start pose", and by the drift corrector. Afterwards the next
     * manual/hold/follow command is sent to Pedro again.
     */
    public void setPose(Pose2 pose) {
        follower.setPose(new Pose(pose.x, pose.y, pose.heading));
        applied = Applied.NONE;
    }

    // ------------------------------------------------------------------------------------- commands

    /**
     * Robot-centric driver control: +forward, +strafe LEFT, +turn counter-clockwise, each -1..1.
     * "Robot-centric" means the directions are relative to the robot's own front, not to the field. The
     * values are fractions of full power, not inches.
     */
    public void manual(double forward, double strafe, double turn) {
        follower.manual(forward, strafe, turn);
        applied = Applied.MANUAL;
        activeRoute = null;
        activeLeg = null;
        lastHold = null;
    }

    /** Holds a pose. Re-issued only when the target actually moves. */
    public void hold(Pose2 target) {
        // Is this the same hold we are already doing? Same kind of command, target within a hair of the old
        // one, and the follower still reports that it is holding (if it is not, the hold must be re-issued).
        boolean same = applied == Applied.HOLD && lastHold != null
                && lastHold.distanceTo(target) <= HOLD_REISSUE_IN
                && Angles.distance(lastHold.heading, target.heading) <= HOLD_REISSUE_RAD
                && follower.holding();
        if (same) {
            return;
        }
        follower.hold(new Pose(target.x, target.y, target.heading));
        applied = Applied.HOLD;
        lastHold = target;
        activeRoute = null;
        activeLeg = null;
    }

    /**
     * Follows a route from the current pose through {@code waypoints} (the last one is the destination),
     * turning gradually toward {@code endHeading}. Issued once per route; the same list object is ignored.
     *
     * @param waypoints    field points to drive through, each {@code {x, y}} in inches; the controller keeps
     *                     handing in the same list instance while the route is unchanged
     * @param endHeadingRad the heading to have at the destination (radians, field frame)
     */
    public void follow(List<double[]> waypoints, double endHeadingRad) {
        follow(waypoints, endHeadingRad, 0.0, 0.0);
    }

    /**
     * Same, with the planner's turn window: keep the start heading for the first {@code turnHoldIn} inches of the
     * route and be done turning for the last {@code turnSettleIn} inches (see {@link HeadingProfile}). The planner
     * checks the route with exactly these numbers, so the robot must be given the same ones.
     */
    public void follow(List<double[]> waypoints, double endHeadingRad, double turnHoldIn, double turnSettleIn) {
        // Same list object = the route we are already driving; do nothing, so the path is not rebuilt (and
        // Pedro's state reset) every loop. A new list object means the controller has planned a new route.
        if (applied == Applied.FOLLOW && waypoints == activeRoute) {
            return;
        }
        Pose2 start = pose();
        if (waypoints.isEmpty()) {
            // No route at all: stay where we are, facing the requested heading.
            hold(new Pose2(start.x, start.y, endHeadingRad));
            return;
        }
        Path path = buildPath(start, waypoints, endHeadingRad, turnHoldIn, turnSettleIn);
        if (path == null) {
            // Nowhere to go (we are already at the destination), so just hold it.
            hold(new Pose2(waypoints.get(waypoints.size() - 1)[0], waypoints.get(waypoints.size() - 1)[1], endHeadingRad));
            return;
        }
        follower.follow(path);
        applied = Applied.FOLLOW;
        activeRoute = waypoints;
        activeLeg = null;
        lastHold = null;
    }

    /**
     * Follows one DRIVE leg of a route: the straight line or Bezier curve of the {@link DrivePath} with the heading rule the leg names, as one Pedro
     * path. This is how the route checker and the drive stay the same thing: the validator sweeps the robot along exactly this
     * {@code DrivePath}, and Pedro's heading interpolators (tangent, reverse tangent, linear, constant, facing a point) are the modes of the leg
     * one to one. Issued once per leg: the same {@code DrivePath} object is ignored while it is being followed.
     *
     * @param speedFraction the fraction (0, 1] of the cruise speed ({@code Auto.CRUISE_SPEED_IN_PER_SEC}) to drive at; below 1 it needs
     *                      {@link #withPathSpeedLimit}
     */
    public void followLeg(DrivePath leg, double speedFraction) {
        if (applied == Applied.LEG && leg == activeLeg) {
            return;
        }
        if (leg.length() < BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN) {
            hold(leg.end());                     // nothing to follow: Pedro cannot follow a zero-length line
            return;
        }
        Path path = legPath(leg);
        if (speedFraction < 1.0 && pathSpeedLimit != null) {
            path = path.with(pathSpeedLimit.at(BioBuzzConstants.Auto.CRUISE_SPEED_IN_PER_SEC * speedFraction));
        }
        follower.follow(path);
        applied = Applied.LEG;
        activeLeg = leg;
        activeRoute = null;
        lastHold = null;
    }

    /** Applies a command of the route runner: hold a pose, drive through waypoints, or follow a DRIVE leg. */
    public void apply(DriveCommand command) {
        switch (command.kind) {
            case WAYPOINTS:
                follow(command.waypoints, command.pose.heading, command.turnHoldIn, command.turnSettleIn);
                break;
            case LEG:
                followLeg(command.leg, command.speedFraction);
                break;
            case HOLD:
            default:
                hold(command.pose);
                break;
        }
    }

    /** Applies whatever the controller asked for this loop. */
    public void apply(AutoShootController.Outputs out, double manualForward, double manualStrafe, double manualTurn) {
        switch (out.driveMode) {
            case FOLLOW:
                follow(out.waypoints, out.holdPose.heading, out.turnHoldIn, out.turnSettleIn);
                break;
            case HOLD:
                hold(out.holdPose);
                break;
            case MANUAL:
            default:
                // Manual is also the fallback, so an unexpected mode hands control back to the driver.
                manual(manualForward, manualStrafe, manualTurn);
                break;
        }
    }

    /** Stops the drive and delivers the zero to the motors. */
    public void stop() {
        follower.stop();
        // update() is what actually writes motor power, so call it once more to push the zero out now.
        follower.update();
        applied = Applied.NONE;
        activeRoute = null;
        activeLeg = null;
        lastHold = null;
    }

    // ------------------------------------------------------------------------------------- paths

    /**
     * The Pedro path of a DRIVE leg: the leg's control points (two = a line, more = a Bezier curve whose control points the path is pulled toward)
     * with Pedro's heading interpolator for the leg's heading mode. The names match one to one ({@code HeadingMode} says so), and
     * {@code PedroLegPathTest} checks it: at every point of the leg Pedro's path has the same place and the same heading as the {@link DrivePath}
     * the route checker swept. (Package-private only so that test can reach it.)
     */
    static Path legPath(DrivePath leg) {
        List<Point2> control = leg.controlPoints();
        Pose[] points = new Pose[control.size()];
        for (int i = 0; i < points.length; i++) {
            points[i] = new Pose(control.get(i).x, control.get(i).y, 0.0);
        }
        Path path = points.length == 2 ? Paths.line(points[0], points[1]) : Paths.curve(points);
        Pose2 start = leg.start();
        Pose2 end = leg.end();
        switch (leg.mode()) {
            case TANGENT:
                return path.tangent();
            case REVERSE_TANGENT:
                return path.reverseTangent();
            case CONSTANT:
                return path.constant(end.heading);
            case FACE_POINT: {
                Interpolator face = Interpolator.facingPoint(new Pose(leg.facePoint().x, leg.facePoint().y, 0.0));
                return path.heading(leg.facesWithBack() ? face.reverse() : face);
            }
            case LINEAR:
            default:
                // the heading turns evenly the SHORT way round, from the heading the leg starts with to the one it ends with
                return path.linear(start.heading, start.heading + Angles.error(start.heading, end.heading));
        }
    }

    /**
     * Builds one Pedro path that starts at {@code start}, passes through the waypoints in order and ends
     * pointing along {@code endHeading}. Returns null if the waypoints add no distance (nothing to follow).
     *
     * <p>Which way the robot faces along the route is NOT decided here: {@link HeadingProfile} decides it, and the
     * route checker ({@code RouteSweep}) asks the same class, so what is checked is exactly what is driven. This
     * method only turns the profile's corners into Pedro lines.
     */
    private static Path buildPath(Pose2 start, List<double[]> waypoints, double endHeading,
                                  double turnHoldIn, double turnSettleIn) {
        HeadingProfile profile = HeadingProfile.build(start, waypoints, endHeading, turnHoldIn, turnSettleIn);
        if (profile.isEmpty()) {
            return null;
        }
        // One straight leg per pair of corners; each leg's start and end headings come from the profile, and
        // Pedro blends between them ("linear" heading interpolation) along the leg.
        Path[] legs = new Path[profile.vertexCount() - 1];
        for (int i = 0; i < legs.length; i++) {
            Pose a = new Pose(profile.x(i), profile.y(i), profile.headingAtVertex(i));
            Pose b = new Pose(profile.x(i + 1), profile.y(i + 1), profile.headingAtVertex(i + 1));
            legs[i] = Paths.line(a, b).linear(a.heading(), b.heading());
        }
        return legs.length == 1 ? legs[0] : Paths.path(legs);
    }
}
