package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The simplest autonomous routine for the RED alliance: it only drives, in two short straight lines, away from its
 * start position beside the red wall. It does not shoot, use the intake or use the camera.
 *
 * <p><b>When to run it.</b> "Autonomous" is the part of a match where the robot drives itself with no driver input.
 * Pick "RedAlliance_LeavePark" in the Driver Station Autonomous list, put the robot on its start pose (below)
 * BEFORE pressing INIT, press INIT, then PLAY. The robot then drives the two lines; press STOP to end it. The
 * routine does not save a pose for the TeleOp.
 *
 * <p><b>Words used here.</b> A {@code LinearOpMode} is an OpMode written as one method, {@code runOpMode()}, that
 * runs from top to bottom: the code before {@code waitForStart()} is the INIT stage and the code after it is the run.
 * Pedro Pathing's <b>Follower</b> drives the robot along a <b>Path</b> and tracks where the robot is using the
 * Pinpoint odometry computer. A <b>Pose</b> is a position plus a heading. The <b>Ivy</b> command library that comes
 * with Pedro describes the routine as a list of steps called <b>Commands</b>: {@code sequential(...)} runs them one
 * after another, and the <b>Scheduler</b> advances them each time {@code Scheduler.execute()} is called, once per loop.
 *
 * <p><b>Field frame and units.</b> Pedro's field frame: inches, origin at the INNER corner where the audience wall meets
 * the red wall (the face of the walls: the field is 141.348 in across, per the official CAD), +x toward the blue side, +y away from the audience. Headings in this file are written in DEGREES
 * ({@code PoseFactory.degrees()}), 0 = facing +x, counter-clockwise positive, so 180 means facing the red wall.
 * (Telemetry prints the heading in radians.)
 *
 * <p><b>What the route does, in order.</b>
 * <ol>
 *   <li>Start at (9.2, 70.3) with heading 180: robot FRONT toward the red wall (the front is about 0.3 inch from it),
 *       close to the middle of the red wall (the field center is y = 70.67).</li>
 *   <li>Path 1: drive in a straight line to (24.9, 83.6): about 16 inches farther from the wall and 13 inches farther
 *       up the field (20.6 inches of travel). The robot turns gradually while it goes, from heading 180 to heading
 *       135.</li>
 *   <li>Path 2: drive in a straight line to (16, 91.5): about 9 inches back toward the red wall and 8 inches
 *       farther up the field (11.9 inches of travel), keeping heading 135. The routine then ends with the robot
 *       standing there.</li>
 * </ol>
 */
@Autonomous(name = "RedAlliance_LeavePark", group = "Autonomous")
public class RedAlliance_LeavePark extends LinearOpMode {

    // Pedro's Follower: drives the robot along paths and tracks its position.
    private Follower follower;

    // Builds Poses from x and y in inches and a heading in DEGREES.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // The start pose and the corner points of the drive. Careful: "path1" below is a POSE (the end of the first
    // line); the method path1() further down builds the first line itself. point2Start is the same point as path1,
    // the start of the second line.
    private final Pose start = poseFactory.of(9.2112, 70.2702, 180);
    private final Pose path1 = poseFactory.of(24.8608, 83.6152, 135);
    private final Pose point2Start = poseFactory.of(24.8608, 83.6152, 135);
    private final Pose point2 = poseFactory.of(16, 91.50, 135);

    // Autonomous routine: the two paths, one after the other. (A follow step finishes when the Follower reaches the
    // END of its path, not when the robot has stopped, so the second line starts straight away.)
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        // Clear any commands left over from an earlier run, build the Follower, and tell it where the robot starts
        // (it cannot know by itself, so the robot must really be placed on the start pose first).
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Wait here for PLAY. Everything above is the INIT stage.
        waitForStart();
        schedule(autoRoutine());

        // Run until STOP: update Pedro (reads the odometry and drives the motors) and advance the command
        // sequence one step, every loop.
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // x and y in inches, heading in radians.
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            // Only while a path is being followed: inches still to go, and which path it is.
            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    // First line: from the start to path1. linear(start, path1) turns the robot's heading steadily from the start
    // heading (180) to the end heading (135) over the length of the line, independent of the direction of travel.
    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    // Second line: from point2Start to point2, with the heading changing from 135 to 135, i.e. staying at 135.
    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }
}
