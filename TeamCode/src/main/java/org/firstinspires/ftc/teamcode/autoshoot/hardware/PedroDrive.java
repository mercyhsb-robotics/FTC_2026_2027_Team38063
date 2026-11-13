package org.firstinspires.ftc.teamcode.autoshoot.hardware;

import java.util.ArrayList;
import java.util.List;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.AutoShootController;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

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
    /** Legs shorter than this are dropped when building a path (a zero-length line is not followable). */
    private static final double MIN_LEG_IN = 0.5;
    /**
     * A hold command is re-issued only if the target moved by more than this (inches / radians). Re-issuing
     * resets Pedro's internal state, so doing it every loop for a target that barely changes would make the
     * robot twitch.
     */
    private static final double HOLD_REISSUE_IN = 0.05;
    private static final double HOLD_REISSUE_RAD = Math.toRadians(0.25);

    // Which kind of command we last gave the follower (NONE = nothing yet, or we just reset the pose).
    private enum Applied { NONE, MANUAL, FOLLOW, HOLD }

    private final Follower follower;
    private Applied applied = Applied.NONE;
    // The waypoint list of the route being followed. Compared by identity (==), see follow().
    private List<double[]> activeRoute;
    // The pose of the last hold command, to tell whether a new hold command really moved the target.
    private Pose2 lastHold;

    public PedroDrive(Follower follower) {
        this.follower = follower;
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
        // Same list object = the route we are already driving; do nothing, so the path is not rebuilt (and
        // Pedro's state reset) every loop. A new list object means the controller has planned a new route.
        if (applied == Applied.FOLLOW && waypoints == activeRoute) {
            return;
        }
        Pose2 start = pose();
        Path path = buildPath(start, waypoints, endHeadingRad);
        if (path == null) {
            // Nowhere to go (we are already at the destination), so just hold it.
            hold(new Pose2(waypoints.get(waypoints.size() - 1)[0], waypoints.get(waypoints.size() - 1)[1], endHeadingRad));
            return;
        }
        follower.follow(path);
        applied = Applied.FOLLOW;
        activeRoute = waypoints;
        lastHold = null;
    }

    /** Applies whatever the controller asked for this loop. */
    public void apply(AutoShootController.Outputs out, double manualForward, double manualStrafe, double manualTurn) {
        switch (out.driveMode) {
            case FOLLOW:
                follow(out.waypoints, out.holdPose.heading);
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
        lastHold = null;
    }

    // ------------------------------------------------------------------------------------- paths

    /**
     * Builds one Pedro path that starts at {@code start}, passes through the waypoints in order and ends
     * pointing along {@code endHeading}. Returns null if the waypoints add no distance (nothing to follow).
     */
    private static Path buildPath(Pose2 start, List<double[]> waypoints, double endHeading) {
        // The points to visit: the start, then each waypoint that is at least MIN_LEG_IN past the previous one.
        List<double[]> pts = new ArrayList<double[]>();
        pts.add(new double[]{start.x, start.y});
        for (double[] w : waypoints) {
            double[] last = pts.get(pts.size() - 1);
            if (Math.hypot(w[0] - last[0], w[1] - last[1]) >= MIN_LEG_IN) {
                pts.add(new double[]{w[0], w[1]});
            }
        }
        if (pts.size() < 2) {
            return null;
        }
        // How far along the whole route each point is (cumulative inches), so the turn can be spread out
        // in proportion to distance travelled.
        double total = 0.0;
        double[] cumulative = new double[pts.size()];
        for (int i = 1; i < pts.size(); i++) {
            total += Math.hypot(pts.get(i)[0] - pts.get(i - 1)[0], pts.get(i)[1] - pts.get(i - 1)[1]);
            cumulative[i] = total;
        }
        // The shortest signed turn (radians) from the start heading to the end heading. Using this instead of
        // (end - start) means the robot never takes the long way round across the +/-PI wrap-around.
        double turn = Angles.error(start.heading, endHeading);
        // One straight leg per pair of points; each leg's start and end headings are the share of the turn
        // done by that point, and Pedro blends between them ("linear" heading interpolation) along the leg.
        Path[] legs = new Path[pts.size() - 1];
        for (int i = 0; i < legs.length; i++) {
            double h0 = start.heading + turn * (cumulative[i] / total);
            double h1 = start.heading + turn * (cumulative[i + 1] / total);
            Pose a = new Pose(pts.get(i)[0], pts.get(i)[1], h0);
            Pose b = new Pose(pts.get(i + 1)[0], pts.get(i + 1)[1], h1);
            legs[i] = Paths.line(a, b).linear(a.heading(), b.heading());
        }
        return legs.length == 1 ? legs[0] : Paths.path(legs);
    }
}
