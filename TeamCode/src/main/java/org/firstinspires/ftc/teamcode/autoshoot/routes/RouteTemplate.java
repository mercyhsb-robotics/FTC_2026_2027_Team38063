package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

/**
 * A route with the START left open: give it a start pose and it builds the whole route (authored in the RED frame, like every route) from that start.
 *
 * <p>Why it exists. The best place to set the robot up depends on what the route does after the start, and what the route does can depend on the
 * start: a robot that starts where it would shoot from shoots without driving first, so the route has no move to a shooting pose. A template says both
 * things at once, "this is the rest of the route" and "this is how it changes with the start", so {@link StartPoseSearch#bestStart} can try every
 * legal start and {@link StartRobustness} can ask what happens when the robot is placed an inch off.
 *
 * <p>The simplest template keeps every step after the start as it is: {@link #keeping(AutoRoute)}.
 */
public interface RouteTemplate {
    /** The whole route (in the frame of the alliance the template is written for) when the robot starts at {@code start}. */
    AutoRoute build(Pose2 start);

    /** The template of a finished route: the same steps from whatever start is asked for, see {@link AutoRoute#withStartPose}. */
    static RouteTemplate keeping(final AutoRoute route) {
        return new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 start) {
                return route.withStartPose(start);
            }
        };
    }
}
