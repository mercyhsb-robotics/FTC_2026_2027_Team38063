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
 */
public final class PedroDrive {
    /** Legs shorter than this are dropped when building a path (a zero-length line is not followable). */
    private static final double MIN_LEG_IN = 0.5;
    /** A hold command is re-issued only if the target moved by more than this (inches / radians). */
    private static final double HOLD_REISSUE_IN = 0.05;
    private static final double HOLD_REISSUE_RAD = Math.toRadians(0.25);

    private enum Applied { NONE, MANUAL, FOLLOW, HOLD }

    private final Follower follower;
    private Applied applied = Applied.NONE;
    private List<double[]> activeRoute;
    private Pose2 lastHold;

    public PedroDrive(Follower follower) {
        this.follower = follower;
    }

    public Follower follower() {
        return follower;
    }

    /** Runs the localizer and the motors; call exactly once per loop, first. */
    public void update() {
        follower.update();
    }

    public Pose2 pose() {
        Pose p = follower.pose();
        return new Pose2(p.x(), p.y(), p.heading());
    }

    /** Translation speed in inches per second. */
    public double speedInPerSec() {
        return Math.hypot(follower.velocity().vx, follower.velocity().vy);
    }

    /** Turn rate in radians per second. */
    public double omegaRadPerSec() {
        return follower.velocity().omega;
    }

    public void setPose(Pose2 pose) {
        follower.setPose(new Pose(pose.x, pose.y, pose.heading));
        applied = Applied.NONE;
    }

    // ------------------------------------------------------------------------------------- commands

    /** Robot-centric driver control: +forward, +strafe LEFT, +turn counter-clockwise, each -1..1. */
    public void manual(double forward, double strafe, double turn) {
        follower.manual(forward, strafe, turn);
        applied = Applied.MANUAL;
        activeRoute = null;
        lastHold = null;
    }

    /** Holds a pose. Re-issued only when the target actually moves. */
    public void hold(Pose2 target) {
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
     */
    public void follow(List<double[]> waypoints, double endHeadingRad) {
        if (applied == Applied.FOLLOW && waypoints == activeRoute) {
            return;
        }
        Pose2 start = pose();
        Path path = buildPath(start, waypoints, endHeadingRad);
        if (path == null) {
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
                manual(manualForward, manualStrafe, manualTurn);
                break;
        }
    }

    /** Stops the drive and delivers the zero to the motors. */
    public void stop() {
        follower.stop();
        follower.update();
        applied = Applied.NONE;
        activeRoute = null;
        lastHold = null;
    }

    // ------------------------------------------------------------------------------------- paths

    private static Path buildPath(Pose2 start, List<double[]> waypoints, double endHeading) {
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
        double total = 0.0;
        double[] cumulative = new double[pts.size()];
        for (int i = 1; i < pts.size(); i++) {
            total += Math.hypot(pts.get(i)[0] - pts.get(i - 1)[0], pts.get(i)[1] - pts.get(i - 1)[1]);
            cumulative[i] = total;
        }
        double turn = Angles.error(start.heading, endHeading);
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
