package org.firstinspires.ftc.teamcode.autonomous.pathtests;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Pedro path test: slides the robot 48 inches (4 feet) straight SIDEWAYS (strafes) to its right and then 48 inches
 * back to the left, to where it started. It is the sideways twin of {@code Test_Forward}. Sideways driving relies on
 * the Pinpoint's sideways-measuring (strafe) pod and on all four mecanum wheels turning the right way, so this is
 * the test that shows whether they work together.
 *
 * <p><b>When to run it.</b> After the Pedro tuning steps in docs/TUNING.md (or after changing the drive or the
 * odometry), before trusting a real autonomous routine. It shows up in the Driver Station Autonomous list as
 * "PedroPathingAuto_Strafe". Put the robot on clear floor with at least 4 feet free on its RIGHT side (more is safer:
 * the robot may overshoot), press INIT, then PLAY. The field walls are not needed: the code simply tells Pedro "the
 * robot is at (0, 0) facing +y", so the numbers below are measured from wherever you put the robot.
 *
 * <p><b>What you should see:</b> the robot, always facing the same way, slides to its right about 4 feet, then back to
 * the left and stops near its start. If it turns while it slides, ends short or long, or does not return to the
 * start, the Pedro settings in {@code pedro/Constants.java} need tuning again. This OpMode shows no telemetry.
 *
 * <p><b>Words used here.</b> "Mecanum" wheels have angled rollers so the robot can <b>strafe</b> (move sideways)
 * without turning. This is an "iterative" OpMode: {@code init()} runs once at INIT, then {@code loop()} runs over and
 * over after PLAY. Pedro's <b>Follower</b> drives along a <b>Path</b> and tracks the robot's position with the Pinpoint
 * odometry computer. A <b>Pose</b> is a position (x, y in inches) plus a heading. Heading is written in DEGREES
 * ({@code PoseFactory.degrees()}): 90 means facing +y, so moving toward +x is moving to the robot's RIGHT.
 *
 * <p>The lines that start with {@code //} and mention "panels" are left over from an optional dashboard library
 * (Panels) that this project does not include, so they are switched off.
 */
@Autonomous(name = "PedroPathingAuto_Strafe", group = "Autonomous")
public class Test_Strafe extends OpMode {
    // Pedro's Follower: drives the robot along paths and tracks its position.
    private Follower follower;
    // Which step the test is on: 0 = ready to slide out, 1 = sliding out (back is next), 2 = slide back started.
    private int pathState;

    // Builds Poses from x and y in inches and a heading in DEGREES.
    private final PoseFactory poses = PoseFactory.degrees();
    // The start (0, 0) and the turn-around point (48, 0), both facing +y (heading 90): the robot keeps facing +y while
    // it moves 48 inches toward +x, which is sideways (to its right).
    private final Pose pose1 = poses.of(0, 0, 90);
    private final Pose pose2 = poses.of(48, 0, 90);
    // The two paths: out to pose2 and back to pose1.
    private Path outbound;
    private Path inbound;

    // Runs once at INIT: builds the Follower, tells it where the robot is, and builds the two paths.
    @Override
    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathState = 0;
        follower = Constants.create(hardwareMap);
        follower.setPose(pose1);
        // A straight line whose heading is changed linearly from the first pose's heading to the second's: both are
        // 90, so the robot keeps facing +y and slides sideways along the line.
        outbound = line(pose1, pose2).linear(pose1, pose2);
        inbound = line(pose2, pose1).linear(pose2, pose1);
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
    }

    // Runs over and over after PLAY. A tiny state machine: when the follower is no longer busy, start the next path.
    @Override
    public void loop() {
        // Every loop: read the odometry and drive the motors. Pedro does nothing unless this is called.
        follower.update();
        switch (pathState) {
            case 0:
                // First time through the follower is idle: start sliding out.
                if (!follower.isBusy()) {
                    follower.follow(outbound);
                    pathState = 1;
                }
                break;
            case 1:
                // The outbound path has finished: slide back.
                if (!follower.isBusy()) {
                    follower.follow(inbound);
                    pathState = 2;
                }
                break;
            default:
                // Finished: nothing more to do.
                break;
        }
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.pose().x());
//        panelsTelemetry.debug("Y", follower.pose().y());
//        panelsTelemetry.debug("Heading", follower.pose().heading());
//        panelsTelemetry.update(telemetry);
    }
}
