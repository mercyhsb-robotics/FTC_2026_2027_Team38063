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
 * Autonomous routine for the RED alliance that starts on tile C1: shoot the starting POLLEN, then drive to the far
 * end of the field.
 *
 * <p><b>When to run it.</b> "Autonomous" is the part of a match where the robot drives itself with no driver input.
 * Pick "RedAlliance_C1" in the Driver Station Autonomous list, put the robot on its start pose (below, in tile C1,
 * FRONT against the audience wall) BEFORE pressing INIT, press INIT, then PLAY. The robot then runs the route on its
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
 * <p><b>Field frame and units.</b> Pedro's field frame: inches, origin at the INNER corner where the audience wall meets
 * the red wall (the face of the walls: the field is 141.348 in across, per the official CAD), +x toward the blue side, +y away from the audience. Headings in this file are written in DEGREES
 * ({@code PoseFactory.degrees()}), 0 = facing +x, counter-clockwise positive, so -90 means facing the audience wall
 * and +90 facing away from it. (Telemetry prints the heading in radians.)
 *
 * <p><b>What the route does, in order.</b>
 * <ol>
 *   <li>Start in tile C1 at (58.0, 9.2) with heading -90: robot FRONT against the audience wall, so the shooter at
 *       the BACK points straight up the field. This is almost the default TeleOp start pose and almost in line with
 *       the red HIVE (its center line is x = 57.92 in the official CAD field), close to the team's measured shot position.</li>
 *   <li>Hold that position and heading (no turn).</li>
 *   <li>Spin the flywheel up to the manual speed ({@code BioBuzzConstants.Shooter.MANUAL_TARGET_TPS}), wait until it
 *       is up to speed, then run the feeder for 5 seconds.</li>
 *   <li>Keep the feeder running for another 4.5 seconds, then stop the flywheel and feeder. (The routine does not
 *       count shots; it just waits long enough for the volley.)</li>
 *   <li>Path 1: drive in a straight line, BACKWARDS (the back of the robot leads), from (58.0, 9.2) to (62.0, 118.1),
 *       about 109 inches up the field to the far end.</li>
 *   <li>Path 2: a short straight line, also backwards, 4 inches toward the red wall to (58.0, 118.1). Because the
 *       robot's heading follows its direction of travel, it turns about 90 degrees here (from facing the audience
 *       wall to facing +x).</li>
 *   <li>Path 3: a straight line, backwards, to (12, 118.1), about 46 inches toward the red wall, ending facing 0
 *       degrees (+x).</li>
 * </ol>
 */
@Autonomous(name = "RedAlliance_C1", group = "Autonomous")
public class
RedAlliance_C1 extends LinearOpMode {
    // The shooter (flywheel and feeder).
    Launcher launcher;

    // Pedro's Follower: drives the robot along paths and tracks its position.
    private Follower follower;

    // Builds Poses from x and y in inches and a heading in DEGREES.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Where the robot starts (heading -90 = front toward the audience wall) and the three waypoints of the drive.
    // The heading written in point1, point2 and point3 is not used by the paths below: reverseTangent() sets the
    // heading from the direction of travel, so only the x and y matter (the robot actually ends facing 0 degrees,
    // not -90). The start heading does matter: it tells Pedro which way the robot is facing at the beginning.
    private final Pose start = poseFactory.of(58.0, 9.2, -90);
    private final Pose point1 = poseFactory.of(62.0, 118.1, -90);
    private final Pose point2 = poseFactory.of(58.0, 118.1, -90);
    private final Pose point3 = poseFactory.of(12, 118.1, -90);

    // A straight line from the start to point1. reverseTangent(): the robot's heading is the direction of travel
    // turned 180 degrees, so it drives BACKWARDS along the line (its back end leads).
    public Path path1() {
        return line(start, point1).reverseTangent();
    }

    // The next straight line (point1 to point2), also driven backwards.
    public Path path2() {return line(point1, point2).reverseTangent();}

    // The last straight line (point2 to point3), also driven backwards.
    public Path path3() {return line(point2, point3).reverseTangent();}

    // Autonomous routine: the whole sequence of steps, built once and handed to the Scheduler in runOpMode().
    public Command autoRoutine() {
        return sequential(
                // Start Pose: hold the start position and the heading of -90 degrees (no turn), so the back of the
                // robot (the shooter) points straight up the field. (hold() only starts the hold and finishes at
                // once, so the next step begins right away.)
                hold(follower, poseFactory.of(
                        start.x(),
                        start.y(),
                        -90
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

                // Go home: the three straight lines below, one after another. (A follow step finishes when the
                // Follower reaches the END of its path, not when the robot has stopped, so the next one starts
                // straight away.)
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
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
