package org.firstinspires.ftc.teamcode.pedro;

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

import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;

@Autonomous(name = "RedAlliance_C1", group = "Autonomous")
public class
RedAlliance_C1 extends LinearOpMode {
    Intake intake;
    Launcher launcher;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.0, 9.2, -90);
    private final Pose point1 = poseFactory.of(58.0, 118.1, -90);
    private final Pose point2 = poseFactory.of(10, 118.1, 0);

    public Path path1() {
        return line(start, point1).reverseTangent();
    }

    public Path path2() {return line(point1, point2).reverseTangent();}

//    public Path path3() {return line(point2, point3).tangent();}

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // Start Pose: Heading is 60 degrees towards Red Alliance hive.
                hold(follower, poseFactory.of(
                        start.x(),
                        start.y(),
                        -90
                )),

                // Detect which side of the hive to shoot into, aim and launch pollen.
                aimAndLaunch(),

                // Wait for 4 seconds
                waitMs(4500),

                // Go to path 1
                follow(follower, path1()),

                // Go to path 2
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        // Initialize launcher and intake
        launcher = new Launcher();
        launcher.init(hardwareMap, telemetry);


        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    /**
     * Intake pollen from the ground.
     */
    public Command intakePollen() {
        telemetry.addLine("Intaking pollen...");
        telemetry.update();
//        intake.startIntake();

        // If pollen count > 4, then stop the intake to save batteries / avoid penalties.

        return null;
    }

    /**
     * Aim and launch pollen and nectar.
     */
    public Command aimAndLaunch() {

        return sequential(

                // Step 1: Calculate target RPM and spin up
                instant(() -> {


                    double distance = getDistanceFromHive();

                    // This is just to test out autonomous for now.   What we really want
                    // is to calculate rpm based on distance using AprilTag
//                    long rpm = calculateRpmBasedOnDistance(distance);
                    long rpm = Launcher.LAUNCHER_TARGET_VELOCITY;

                    launcher.increaseRpm(rpm);
                }),

                // Step 2: Wait until launcher is ready
                waitUntil(() -> launcher.isAtTargetRpm()),

                // Step 3: Launch game piece
                instant(() -> {
                    telemetry.addLine("Launching game piece...");
                    telemetry.update();
                    launcher.launch(true);
                    waitMs(1000);
                    launcher.feederForward();
                }),

                // Step 4: Allow time for launching.
                // Ideally we can detect when the launcher has launched four pollen.
                // For now, wait 3 seconds.
                waitMs(5000),

                // Step 5: Stop launcher
                instant(() -> {
                    launcher.stop();
                })
        );
    }

    private long calculateRpmBasedOnDistance(double distance) {
        // Add code to calculate rpm based on distance
        return -1;
    }

    private double getDistanceFromHive() {
        // Add code to get distance from hive
        return -1;
    }


}
