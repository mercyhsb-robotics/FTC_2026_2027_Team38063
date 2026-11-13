package org.firstinspires.ftc.teamcode.autonomous.pathtests;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Pedro path test: drives the robot around a big square (about 91 inches on a side) and back to almost exactly where
 * it started. Use it to see how well Pedro and the Pinpoint odometry hold up over a long drive with four corners.
 *
 * <p><b>When to run it.</b> After the Pedro tuning steps in docs/START_HERE.md, before trusting a real autonomous
 * routine. It shows up in the Driver Station Autonomous list as "AutoPath" (the name in the annotation; the class is
 * called Test_Square). The square is about 91 inches (7.6 feet) on a side, so clear a space at least that big plus
 * room to overshoot (the whole field works). Put the robot on the start pose below (about 27 inches from both the red
 * wall and the audience wall, facing +y) BEFORE pressing INIT, press INIT, then PLAY. Press STOP to end the test.
 *
 * <p><b>What you should see:</b> the robot drives the four sides of the square one after another, always BACKWARDS
 * (its back end leads), and finishes about 1 inch from where it started. If it ends far from the start, or the
 * corners are cut or overshoot badly, the Pedro settings in {@code pedro/Constants.java} need tuning again. Telemetry
 * shows the position and which path is running.
 *
 * <p><b>Words used here.</b> A {@code LinearOpMode} is an OpMode written as one method, {@code runOpMode()}, that
 * runs from top to bottom: the code before {@code waitForStart()} is the INIT stage and the code after it is the run.
 * Pedro's <b>Follower</b> drives along a <b>Path</b> and tracks the robot's position with the Pinpoint odometry
 * computer. A <b>Pose</b> is a position plus a heading. The <b>Ivy</b> command library that comes with Pedro describes
 * the routine as a list of steps called <b>Commands</b>: {@code sequential(...)} runs them one after another, and the
 * <b>Scheduler</b> advances them each time {@code Scheduler.execute()} is called, once per loop.
 *
 * <p><b>Field frame and units.</b> Inches, origin at the corner where the audience wall meets the red wall, +x toward
 * the blue side, +y away from the audience. Headings are written in DEGREES ({@code PoseFactory.degrees()}), 0 =
 * facing +x, counter-clockwise positive. (Telemetry prints the heading in radians.)
 *
 * <p><b>What the route does, in order.</b>
 * <ol>
 *   <li>Start at (26.7, 27.4), heading 90 (facing +y).</li>
 *   <li>Path 1: drive to (27.0, 115.2), about 88 inches toward +y, holding a constant heading of 270 degrees (facing
 *       -y). The robot therefore turns half a turn at the start and then drives BACKWARDS up the field.</li>
 *   <li>Path 2: drive to (117.9, 115.6), about 91 inches toward +x (toward the blue side). BACKWARDS: the heading
 *       follows the direction of travel plus 180 degrees, so the robot faces about 180 degrees (toward the red
 *       wall).</li>
 *   <li>Path 3: drive to (118.0, 25.0), about 91 inches toward -y (back toward the audience wall), backwards,
 *       facing about 90 degrees.</li>
 *   <li>Path 4: drive to (27.3, 26.3), about 91 inches toward -x (back toward the red wall), backwards, facing
 *       about 0 degrees. That ends about 1 inch from where the robot started.</li>
 * </ol>
 */
@Autonomous(name = "AutoPath", group = "Autonomous")
public class Test_Square extends LinearOpMode {

    // Pedro's Follower: drives the robot along paths and tracks its position.
    private Follower follower;

    // Builds Poses from x and y in inches and a heading in DEGREES.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // The start and the four corners of the square. The heading of point2, point3 and point4 is not used (their paths
    // use reverseTangent(), which takes the heading from the direction of travel); point1's heading is used by
    // path1(), which holds it. The start heading tells Pedro which way the robot faces at the beginning.
    private final Pose start = poseFactory.of(26.7238, 27.3536, 90);
    private final Pose point1 = poseFactory.of(26.9781, 115.2414, 270);
    private final Pose point2 = poseFactory.of(117.9191, 115.629, -179.7558);
    private final Pose point3 = poseFactory.of(118.0488, 24.9658, 90.082);
    private final Pose point4 = poseFactory.of(27.2517, 26.3298, -0.8607);

    // Autonomous routine: the four paths, one after the other. (A follow step finishes when the Follower reaches the
    // END of its path, not when the robot has stopped, so the next side starts straight away.)
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
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

    // Side 1: a straight line from the start to point1 with the heading held constant at point1's heading (270
    // degrees).
    public Path path1() {
        return line(start, point1).constant(point1);
    }

    // Side 2: a straight line from point1 to point2, driven backwards (heading = direction of travel + 180 degrees).
    public Path path2() {
        return line(point1, point2).reverseTangent();
    }

    // Side 3: point2 to point3, backwards.
    public Path path3() {
        return line(point2, point3).reverseTangent();
    }

    // Side 4: point3 to point4, backwards.
    public Path path4() {
        return line(point3, point4).reverseTangent();
    }
}
