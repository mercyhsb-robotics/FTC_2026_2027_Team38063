package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Autonomous routine for the RED alliance that starts on tile B1: shoot the starting POLLEN, then drive away.
 *
 * <p><b>When to run it.</b> "Autonomous" is the part of a match where the robot drives itself with no driver input.
 * Pick "RedAlliance_B1" in the Driver Station Autonomous list, put the robot on its start pose (below, in tile B1,
 * FRONT toward the audience wall) BEFORE pressing INIT, press INIT, then PLAY. The robot then runs the route on its
 * own; press STOP to end it. The routine does not use the camera and does not save a pose for the TeleOp.
 *
 * <p><b>Words used here.</b> A {@code LinearOpMode} is an OpMode written as one method, {@code runOpMode()}, that
 * runs from top to bottom: the code before {@code waitForStart()} is the INIT stage and the code after it is the run.
 * Pedro Pathing's <b>Follower</b> drives the robot along a <b>Path</b> (or holds a <b>Pose</b>, which is a position
 * plus a heading) and tracks where the robot is using the Pinpoint odometry computer. The <b>Ivy</b> command library
 * that comes with Pedro describes the routine as a list of steps called <b>Commands</b>: {@code sequential(...)} runs
 * them one after another, and the <b>Scheduler</b> advances them each time {@code Scheduler.execute()} is called, once
 * per loop.
 *
 * <p>The <b>flywheel</b> is the fast-spinning wheel that throws a POLLEN out the back of the robot and the
 * <b>feeder</b> is the servo that pushes each POLLEN into it (both are in the {@code Launcher} class). Flywheel speed
 * is measured in encoder <b>ticks</b> per second (a tick is one count of the motor's encoder).
 *
 * <p><b>Field frame and units.</b> Pedro's field frame: inches, origin at the corner where the audience wall meets
 * the red wall, +x toward the blue side, +y away from the audience. Headings in this file are written in DEGREES
 * ({@code PoseFactory.degrees()}), 0 = facing +x, counter-clockwise positive, so -90 means facing the audience wall
 * and +90 facing away from it. (Telemetry prints the heading in radians.)
 *
 * <p><b>What the route does, in order.</b>
 * <ol>
 *   <li>Start in tile B1 at (35.0, 9.2) with heading -90: robot FRONT toward the audience wall, so the shooter at
 *       the BACK points up the field.</li>
 *   <li>Hold that position and turn in place to heading -115 degrees. That points the back of the robot (the
 *       shooter) about 65 degrees, roughly toward the red HIVE.</li>
 *   <li>Spin the flywheel up to the manual speed ({@code BioBuzzConstants.Shooter.MANUAL_TARGET_TPS}), wait until it
 *       is up to speed, then run the feeder for 5 seconds.</li>
 *   <li>Keep the feeder running for another 4.5 seconds, then stop the flywheel and feeder. (The routine does not
 *       count shots; it just waits long enough for the volley.)</li>
 *   <li>Path 1: drive in a straight line from (35.0, 9.2) to (35.8, 90), about 81 inches up the field, BACKWARDS
 *       (the back of the robot leads).</li>
 *   <li>Path 2: drive in a straight line, backwards, to (12, 90), about 24 inches toward the red wall. The robot
 *       ends there facing 0 degrees (+x). (The HIVE frame spans about y = 52.5 to 91.5, so this is level with its
 *       far end.)</li>
 * </ol>
 */
@Autonomous(name = "RedAlliance_B1", group = "Autonomous")
public class
RedAlliance_B1 extends LinearOpMode {
    // The shooter (flywheel and feeder).
    Launcher launcher;

    // Pedro's Follower: drives the robot along paths and tracks its position.
    private Follower follower;

    // Builds Poses from x and y in inches and a heading in DEGREES.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Where the robot starts (heading -90 = front toward the audience wall) and the two waypoints of the drive.
    // The heading of point1 and point2 is not used by the paths below: reverseTangent() sets the heading from the
    // direction of travel, so only the x and y matter. The start heading does matter: it tells Pedro which way the
    // robot is facing at the beginning.
    private final Pose start = poseFactory.of(35.0, 9.2, -90);
    private final Pose point1 = poseFactory.of(35.8, 90, -90);
    private final Pose point2 = poseFactory.of(12, 90, 0);

    // A straight line from the start to point1. reverseTangent(): the robot's heading is the direction of travel
    // turned 180 degrees, so it drives BACKWARDS along the line (its back end leads).
    public Path path1() {
        return line(start, point1).reverseTangent();
    }

    // A straight line from point1 to point2, also driven backwards.
    public Path path2() {return line(point1, point2).reverseTangent();}

//    public Path path3() {return line(point2, point3).tangent();}

    // Autonomous routine: the whole sequence of steps, built once and handed to the Scheduler in runOpMode().
    public Command autoRoutine() {
        return sequential(
                // Start Pose: hold the start position and turn in place to heading -115 degrees. That makes the back
                // of the robot (the shooter) point about 65 degrees, roughly toward the red HIVE. (hold() only
                // starts the hold and finishes at once, so the next step begins while the robot is still turning.)
                hold(follower, poseFactory.of(
                        start.x(),
                        start.y(),
                        -115
                )),

                // Spin up and launch the pollen (see aimAndLaunch). It does NOT detect which side of the HIVE is up:
                // it fires at one fixed flywheel speed.
                aimAndLaunch(),

                // Wait 4.5 seconds (4500 milliseconds) while the feeder keeps running, so the rest of the POLLEN
                // leave.
                waitMs(4500),

                // Switch the flywheel and the feeder off before driving.
                instant(() -> {
                    launcher.stop();
                }),

                // Go to path 1. (A follow step finishes when the Follower reaches the END of its path, not when the
                // robot has stopped, so the next path starts straight away.)
                follow(follower, path1()),

                // Go to path 2
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        // Set up the launcher (shooter). This routine does not use the intake.
        launcher = new Launcher();
        launcher.init(hardwareMap, telemetry);


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

    /**
     * Aim and launch pollen and nectar.
     *
     * <p>Despite the name it does not aim: the robot's heading was already set by the hold step in autoRoutine().
     * It spins the flywheel up to a fixed speed, waits for it, and feeds for a fixed time; it neither counts shots
     * nor reads the camera.
     */
    public Command aimAndLaunch() {

        return sequential(

                // Step 1: Spin up the flywheel to a fixed speed (BioBuzzConstants.Shooter.MANUAL_TARGET_TPS, in ticks
                // per second; increaseRpm takes ticks per second despite its name).
                // TODO: calculate the rpm from the AprilTag distance instead of a fixed velocity.
                instant(() -> launcher.increaseRpm(BioBuzzConstants.Shooter.MANUAL_TARGET_TPS)),

                // Step 2: Wait until launcher is ready (the flywheel has reached the lower edge of its speed window,
                // or the spin-up timeout has passed, so this can never wait forever).
                waitUntil(() -> launcher.isAtTargetRpm()),

                // Step 3: Launch game piece. launch(true) keeps the manual speed and starts the feeder if the
                // flywheel is fast enough; feederForward() then makes sure the feeder runs regardless.
                instant(() -> {
                    telemetry.addLine("Launching game piece...");
                    telemetry.update();
                    launcher.launch(true);
                    launcher.feederForward();
                }),

                // Step 4: Allow time for launching.
                // Ideally we can detect when the launcher has launched four pollen.
                // For now, wait 5 seconds (5000 milliseconds).
                waitMs(5000)
        );
    }

}
