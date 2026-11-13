package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CameraModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.ConstantsCheck;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.GamepadButtons;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.LimelightVision;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.PedroDrive;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.RouteRunnerHardware;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.run.RoutePicker;
import org.firstinspires.ftc.teamcode.autoshoot.run.RouteExecutor;
import org.firstinspires.ftc.teamcode.autoshoot.run.RunSettingsCheck;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The autonomous OpMode that runs a ROUTE from the route catalog ({@code RouteCatalog}): the student chooses the route and the alliance with the
 * gamepad during INIT, the screen says everything that is known about the choice, and on PLAY the robot drives it with the route runner.
 *
 * <p><b>How to use it.</b> Pick "Auto Route Picker" in the Driver Station Autonomous list and press INIT. With gamepad 1: the D-pad up / down chooses
 * the route (the list wraps round), D-pad left chooses RED and D-pad right BLUE (the buttons are in {@code BioBuzzConstants.Run}). The screen
 * shows the route and what it assumes (does the partner tip the HIVE first?), how long it should take, how many points are ours, whether the route
 * checker found a problem, the words that say WHERE TO PUT THE ROBOT (measured from the field walls to the robot's EDGES), the Limelight's state and the
 * values that are still waiting for a measurement. Put the robot where it says, and press PLAY.
 *
 * <p><b>When it refuses.</b> A route with an ERROR from the route checker will not start, and neither will a route that waits for the HIVE to tip
 * while the Limelight is not working: the last line of the screen says "REFUSED" and why, and after PLAY the robot stays still.
 *
 * <p><b>What happens on PLAY.</b> {@link RouteExecutor} runs the route (plain Java, tested without a robot); {@link RouteRunnerHardware} carries its
 * commands out with Pedro, the launcher, the intake and the camera. A clock stops everything at the end of the AUTO period. When the route is over
 * and when the OpMode stops, the final pose, the alliance and what the robot believes about the HIVE are saved ({@code PoseStore}), so the TeleOp can
 * carry on from where the robot stands. This OpMode never touches the old hand-written ones (RedAlliance_B1, _C1, _LeavePark).
 *
 * <p>Words: an <b>iterative OpMode</b> is one whose {@code init_loop()} runs over and over while the drivers wait (here: the menu), then
 * {@code start()} once at PLAY and {@code loop()} over and over until the end. A <b>route</b> is a short list of steps (start, drive, intake, shoot,
 * wait, park) kept as data; see docs/AUTOS.md.
 */
@Autonomous(name = "Auto Route Picker", group = "2 Autonomous")
public class AutoRoutePicker extends OpMode {
    // ---- the robot
    private Launcher launcher;
    private Intake intake;
    private CRServo servoLeft;
    private CRServo servoRight;
    private PedroDrive drive;
    private String driveNote = "";
    private LimelightVision vision;
    private String visionNote = "";
    private RouteRunnerHardware hardware;

    // ---- the choice and the run
    private RoutePicker picker;
    private ConstantsCheck.Report constantsReport;
    private Alliance shownAlliance;
    private RouteExecutor executor;
    private boolean running;
    private boolean savedAtEnd;
    private String refusedBecause = "";

    // ---- clocks (nanoseconds from System.nanoTime())
    private long initNanos;
    private long startNanos;

    /** INIT: checks the constants, finds the hardware and builds the menu. */
    @Override
    public void init() {
        constantsReport = ConstantsCheck.run();
        ConstantsCheck.Report runReport = RunSettingsCheck.run();
        if (!constantsReport.ok() || !runReport.ok()) {
            throw new IllegalStateException("BioBuzzConstants has errors: " + constantsReport.errors + " " + runReport.errors);
        }
        constantsReport.warnings.addAll(runReport.warnings);
        initNanos = System.nanoTime();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        launcher = new Launcher();
        launcher.init(hardwareMap, telemetry);
        intake = new Intake();
        intake.init(hardwareMap);
        servoLeft = hardwareMap.get(CRServo.class, BioBuzzConstants.Hardware.LEFT_INTAKE_SERVO);
        servoRight = hardwareMap.get(CRServo.class, BioBuzzConstants.Hardware.RIGHT_INTAKE_SERVO);

        // The odometry is not optional here: without it no route can be driven.
        try {
            drive = new PedroDrive(Constants.create(hardwareMap)).withPathSpeedLimit(Constants.foresightConfig.maxPathSpeed);
            driveNote = "Pedro + Pinpoint";
        } catch (RuntimeException e) {
            drive = null;
            driveNote = "NO DRIVE (Pedro/Pinpoint failed: " + e.getMessage() + ")";
        }
        try {
            vision = new LimelightVision(hardwareMap, CameraModel.fromConstants());
            vision.start();
            visionNote = "Limelight started";
        } catch (RuntimeException e) {
            vision = null;
            visionNote = "NO Limelight (" + e.getMessage() + ")";
        }
        if (drive != null) {
            hardware = new RouteRunnerHardware(drive, launcher, intake, servoLeft, servoRight, vision);
        }
        picker = new RoutePicker(RouteCatalog.all(), Alliance.RED);
        showAllianceColor();
    }

    /** INIT loop: the student chooses, the screen says everything about the choice. */
    @Override
    public void init_loop() {
        picker.buttons(
                GamepadButtons.pressed(gamepad1, BioBuzzConstants.Run.MENU_PREVIOUS_ROUTE_BUTTON),
                GamepadButtons.pressed(gamepad1, BioBuzzConstants.Run.MENU_NEXT_ROUTE_BUTTON),
                GamepadButtons.pressed(gamepad1, BioBuzzConstants.Run.MENU_RED_BUTTON),
                GamepadButtons.pressed(gamepad1, BioBuzzConstants.Run.MENU_BLUE_BUTTON));
        if (picker.alliance() != shownAlliance) {
            showAllianceColor();
        }
        VisionFrame.Status camera = visionStatus();
        telemetry.addData("Drive", driveNote);
        telemetry.addData("Vision", visionNote);
        for (String line : picker.screen(camera, constantsReport.placeholders)) {
            telemetry.addLine(line);
        }
        if (drive == null) {
            telemetry.addLine(">>> NO ODOMETRY: " + driveNote + " - the robot will not start");
        }
        for (String w : constantsReport.warnings) {
            telemetry.addLine("Constants WARNING: " + w);
        }
        telemetry.update();
    }

    /** PLAY: if the choice is allowed, builds the runner and starts the clock. */
    @Override
    public void start() {
        startNanos = System.nanoTime();
        RoutePicker.Readiness ready = picker.readiness(visionStatus());
        if (drive == null) {
            refusedBecause = "there is no odometry: " + driveNote;
        } else if (!ready.canStart()) {
            refusedBecause = ready.refusal;
        }
        if (!refusedBecause.isEmpty()) {
            running = false;
            return;
        }
        executor = new RouteExecutor(picker.choice().analysis);
        running = true;
    }

    /** The loop: sense, decide, act, report. */
    @Override
    public void loop() {
        if (!running) {
            telemetry.addLine(">>> REFUSED, THE ROBOT STAYS STILL: " + refusedBecause);
            telemetry.update();
            return;
        }
        RouteExecutor.Inputs in = hardware.sense(secondsSince(startNanos), secondsSince(initNanos));
        RouteExecutor.Outputs out = executor.update(in);
        hardware.act(out);

        if (out.finished && !savedAtEnd) {
            executor.saveForTeleOp(in.pose, System.nanoTime());
            savedAtEnd = true;
        }
        telemetry.addData("Route", picker.choice().route.title() + " (" + picker.alliance() + ")");
        telemetry.addLine(out.status.line());
        telemetry.addData("Doing", out.status.doing);
        telemetry.addData("Pose", in.pose);
        telemetry.addData("HIVE up side", executor.tracker().upSide() + (executor.tracker().confirmed() ? " (seen)" : " (not confirmed)")
                + "   camera: " + in.frame.status);
        if (!out.status.stoppedBy.isEmpty()) {
            telemetry.addData("STOPPED", out.status.stoppedBy);
        }
        for (String problem : out.status.problems) {
            telemetry.addLine("PROBLEM: " + problem);
        }
        telemetry.update();
    }

    /** STOP: everything off, and the pose saved for the TeleOp. */
    @Override
    public void stop() {
        if (hardware != null) {
            hardware.stopAll();
        }
        if (running && executor != null && drive != null) {
            executor.saveForTeleOp(drive.pose(), System.nanoTime());
        }
    }

    // ---------------------------------------------------------------------------------------------

    /** What the Limelight reports right now (a missing camera reports NOT_CONNECTED). */
    private VisionFrame.Status visionStatus() {
        VisionFrame frame = hardware != null ? hardware.camera(secondsSince(initNanos))
                : VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, secondsSince(initNanos));
        return frame.status;
    }

    private static double secondsSince(long nanos) {
        return (System.nanoTime() - nanos) / 1.0e9;
    }

    /** Lights the gamepad's light bar in the alliance color so the driver can see the choice at a glance. */
    private void showAllianceColor() {
        shownAlliance = picker.alliance();
        double[] c = shownAlliance == Alliance.RED ? BioBuzzConstants.Controls.RED_LED : BioBuzzConstants.Controls.BLUE_LED;
        try {
            gamepad1.setLedColor(c[0], c[1], c[2], Gamepad.LED_DURATION_CONTINUOUS);
        } catch (RuntimeException ignored) {
            // not every controller has a light
        }
    }
}
