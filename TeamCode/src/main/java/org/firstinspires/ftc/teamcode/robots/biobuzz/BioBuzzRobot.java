package org.firstinspires.ftc.teamcode.robots.biobuzz;

import java.util.Collections;
import java.util.List;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.AutoShootController;
import org.firstinspires.ftc.teamcode.autoshoot.core.CameraModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.ClusterAnalyzer;
import org.firstinspires.ftc.teamcode.autoshoot.core.ConstantsCheck;
import org.firstinspires.ftc.teamcode.autoshoot.core.ClusterView;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.HiveTracker;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.PoseStore;
import org.firstinspires.ftc.teamcode.autoshoot.core.ScoringPlanner;
import org.firstinspires.ftc.teamcode.autoshoot.core.ShotTable;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.GamepadButtons;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.LimelightVision;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.PedroDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The driver-controlled OpMode.
 *
 * <p>HOLD the shoot button (BioBuzzConstants.Controls.SHOOT_BUTTON) and the robot drives to the closest
 * feasible scoring pose for whichever cell of its HIVE is tipped up, aligns on the AprilTags, and fires
 * every POLLEN it holds; release it and you are driving again. Everything else is the manual robot.
 *
 * <p>Choose the alliance during INIT with the red/blue buttons. If the Pinpoint or Pedro cannot start,
 * the robot still drives with the legacy mecanum code and the automatic shooting is disabled; if the
 * Limelight is missing the robot shoots from its belief about the HIVE and the driver's word.
 *
 * <p>This class is wiring only. The behaviour lives in {@code autoshoot.core} and every tunable value in
 * {@link BioBuzzConstants}.
 */
@TeleOp
public class BioBuzzRobot extends OpMode {
    // ---- mechanisms
    private Launcher launcher;
    private Intake intake;
    private CRServo servoLeft;
    private CRServo servoRight;

    // ---- drive: Pedro owns the motors when it starts; otherwise the legacy class does
    private PedroDrive drive;
    private MecanumDrive legacyDrive;
    private String driveNote = "";

    // ---- vision
    private LimelightVision vision;
    private String visionNote = "";

    // ---- logic
    private final FieldModel field = new FieldModel();
    private ShotTable table;
    private ConstantsCheck.Report constantsReport;
    private final CameraModel camera = CameraModel.fromConstants();
    private ClusterAnalyzer analyzer;
    private ScoringPlanner planner;
    private Alliance alliance = Alliance.RED;
    private HiveTracker tracker;
    private AutoShootController controller;

    // ---- state
    private long startNanos;
    private boolean poseTrusted;
    private String poseSource = "";
    private boolean prevToggle;
    private double resetHeldSinceSec = Double.NaN;
    private boolean resetLatched;
    private AutoShootController.State prevState = AutoShootController.State.IDLE;
    private boolean manualLaunchOn;
    private AutoShootController.Outputs lastOut;
    private List<ClusterView> lastViews = Collections.emptyList();
    private VisionFrame lastFrame;

    @Override
    public void init() {
        // Refuse nonsense in BioBuzzConstants here, with a message that names the problem, instead of failing later.
        constantsReport = ConstantsCheck.run();
        if (!constantsReport.ok()) {
            throw new IllegalStateException("BioBuzzConstants has errors: " + constantsReport.errors);
        }
        table = ShotTable.fromConstants();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        launcher = new Launcher();
        launcher.init(hardwareMap, telemetry);
        intake = new Intake();
        intake.init(hardwareMap);
        servoLeft = hardwareMap.get(CRServo.class, BioBuzzConstants.Hardware.LEFT_INTAKE_SERVO);
        servoRight = hardwareMap.get(CRServo.class, BioBuzzConstants.Hardware.RIGHT_INTAKE_SERVO);

        // Pedro builds the Pinpoint localizer first, so if that fails no drive motor has been touched yet.
        try {
            drive = new PedroDrive(Constants.create(hardwareMap));
            driveNote = "Pedro + Pinpoint";
        } catch (RuntimeException e) {
            drive = null;
            driveNote = "LEGACY drive (Pedro/Pinpoint failed: " + e.getMessage() + ")";
            legacyDrive = new MecanumDrive();
            legacyDrive.init(hardwareMap);
        }

        try {
            vision = new LimelightVision(hardwareMap, camera);
            vision.start();
            visionNote = "Limelight started";
        } catch (RuntimeException e) {
            vision = null;
            visionNote = "NO Limelight (" + e.getMessage() + ")";
        }

        analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());
        planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(table));

        PoseStore.Snapshot snap = PoseStore.load(System.nanoTime(), BioBuzzConstants.Controls.POSE_CARRYOVER_MAX_AGE_SEC);
        if (snap != null) {
            alliance = snap.alliance;
        }
        showAllianceColor();
    }

    @Override
    public void init_loop() {
        if (GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.ALLIANCE_RED_BUTTON) && alliance != Alliance.RED) {
            alliance = Alliance.RED;
            showAllianceColor();
        } else if (GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.ALLIANCE_BLUE_BUTTON) && alliance != Alliance.BLUE) {
            alliance = Alliance.BLUE;
            showAllianceColor();
        }
        telemetry.addData("ALLIANCE", alliance + "  (" + BioBuzzConstants.Controls.ALLIANCE_RED_BUTTON + " = red, "
                + BioBuzzConstants.Controls.ALLIANCE_BLUE_BUTTON + " = blue)");
        telemetry.addData("Drive", driveNote);
        if (!constantsReport.warnings.isEmpty()) {
            telemetry.addData("Constants WARNINGS", constantsReport.warnings);
        }
        telemetry.addData("Placeholders still to measure", constantsReport.placeholders.size() + ": " + constantsReport.placeholders);
        telemetry.addData("Vision", visionNote + (vision != null ? "  frames=" + vision.framesSeen() + " link=" + vision.connected() : ""));
        PoseStore.Snapshot snap = PoseStore.load(System.nanoTime(), BioBuzzConstants.Controls.POSE_CARRYOVER_MAX_AGE_SEC);
        telemetry.addData("Start pose", snap != null && snap.alliance == alliance
                ? "carried over from the previous OpMode" : "default: center of tile C1, front against the audience wall");
    }

    @Override
    public void start() {
        startNanos = System.nanoTime();
        tracker = new HiveTracker(field, alliance, HiveTracker.Config.fromConstants());
        controller = new AutoShootController(alliance, field, planner, table, tracker,
                AutoShootController.Config.fromConstants());

        PoseStore.Snapshot snap = PoseStore.load(System.nanoTime(), BioBuzzConstants.Controls.POSE_CARRYOVER_MAX_AGE_SEC);
        Pose2 startPose;
        if (snap != null && snap.alliance == alliance) {
            startPose = snap.pose;
            tracker.reset(snap.hiveUpSide, snap.hiveConfirmed);
            poseSource = "carried over from the previous OpMode";
        } else {
            startPose = defaultStartPose();
            poseSource = "default start pose (tile C1, front on the audience wall)";
        }
        if (drive != null) {
            drive.setPose(startPose);
            poseTrusted = true;
        }
    }

    private Pose2 defaultStartPose() {
        Pose2 red = Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN,
                BioBuzzConstants.Controls.DEFAULT_START_Y_IN, BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG);
        return alliance.fromRed(red, field.size());
    }

    private void showAllianceColor() {
        double[] c = alliance == Alliance.RED ? BioBuzzConstants.Controls.RED_LED : BioBuzzConstants.Controls.BLUE_LED;
        try {
            gamepad1.setLedColor(c[0], c[1], c[2], Gamepad.LED_DURATION_CONTINUOUS);
        } catch (RuntimeException ignored) {
            // not every controller has a light
        }
    }

    private double nowSec() {
        return (System.nanoTime() - startNanos) / 1.0e9;
    }

    // ---------------------------------------------------------------------------------------------

    @Override
    public void loop() {
        double now = nowSec();

        // 1. Sense.
        if (drive != null) {
            drive.update();
        }
        Pose2 pose = drive != null ? drive.pose() : null;
        VisionFrame frame = vision != null ? vision.read(now)
                : VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, now);
        List<ClusterView> views = pose != null ? analyzer.analyze(alliance, pose.heading, frame.tags)
                : Collections.<ClusterView>emptyList();
        if (pose != null) {
            tracker.update(now, frame, views, HiveTracker.sideOf(pose, field));
        }
        lastFrame = frame;
        lastViews = views;

        // 2. Driver inputs.
        double[] sticks = readSticks();
        boolean toggle = GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.TOGGLE_HIVE_BUTTON);
        if (toggle && !prevToggle) {
            tracker.toggle();
            gamepad1.rumble(BioBuzzConstants.Controls.RUMBLE_STARTED_MS);
        }
        prevToggle = toggle;
        boolean shootHeld = GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.SHOOT_BUTTON);
        handlePoseReset(now);

        // 3. Decide.
        AutoShootController.Outputs out = null;
        if (drive != null) {
            AutoShootController.Inputs in = new AutoShootController.Inputs();
            in.nowSec = now;
            in.shootHeld = shootHeld;
            in.pose = pose;
            in.speedInS = drive.speedInPerSec();
            in.omegaRadS = drive.omegaRadPerSec();
            in.poseTrusted = poseTrusted;
            in.flywheelTps = launcher.velocity();
            in.frame = frame;
            in.views = views;
            in.stickMagnitude = sticks[3];
            out = controller.update(in);
            lastOut = out;
        }

        // 4. Act.
        boolean automatic = out != null && out.state != AutoShootController.State.IDLE;
        if (automatic) {
            intake.stopIntake();                    // never intake while shooting: the robot may hold at most 4
            servoLeft.setPower(0);
            servoRight.setPower(0);
            launcher.setTargetVelocity(out.flywheelTargetTps);
            launcher.setFeederOn(out.feederOn);
            drive.apply(out, sticks[0], sticks[1], sticks[2]);
            manualLaunchOn = false;
        } else {
            if (prevState != AutoShootController.State.IDLE) {
                launcher.stop();                    // the button was just released: stop the flywheel and feeder
            }
            if (drive != null) {
                drive.apply(out, sticks[0], sticks[1], sticks[2]);
            } else {
                legacyDrive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            manualControls();
        }
        if (out != null) {
            prevState = out.state;
            giveFeedback(out);
        }

        // 5. Report.
        report(out, pose);
    }

    /** {forward, strafe, turn, translation stick magnitude}: forward/strafe/turn are ready for the drive. */
    private double[] readSticks() {
        double lx = -gamepad1.left_stick_x;       // stick right = move right = negative strafe (+strafe is LEFT)
        double ly = -gamepad1.left_stick_y;       // stick up is negative on the SDK; forward is positive
        double rx = -gamepad1.right_stick_x;      // stick right = clockwise = negative turn (+turn is counter-clockwise)
        double mag = Math.hypot(lx, ly);
        double db = BioBuzzConstants.Controls.STICK_DEADBAND;
        double scale = mag < db ? 0.0 : (mag - db) / (1.0 - db) / mag;
        double fwd = ly * scale * BioBuzzConstants.Controls.MANUAL_FORWARD_SIGN * BioBuzzConstants.Controls.MANUAL_DRIVE_SCALE;
        double str = lx * scale * BioBuzzConstants.Controls.MANUAL_STRAFE_SIGN * BioBuzzConstants.Controls.MANUAL_DRIVE_SCALE;
        double turn = Math.abs(rx) < db ? 0.0 : rx * BioBuzzConstants.Controls.MANUAL_TURN_SIGN * BioBuzzConstants.Controls.MANUAL_TURN_SCALE;
        return new double[]{fwd, str, turn, Math.min(1.0, mag)};
    }

    /** The existing manual launcher, intake and corner-servo controls. */
    private void manualControls() {
        // Corner servos on the triggers.
        servoLeft.setPower(gamepad1.left_trigger > BioBuzzConstants.Controls.TRIGGER_THRESHOLD ? -1 : 0);
        servoRight.setPower(gamepad1.right_trigger > BioBuzzConstants.Controls.TRIGGER_THRESHOLD ? 1 : 0);

        // Launcher: triangle spins up and feeds, square stops; the bumpers run the feeder only while held.
        if (gamepad1.triangle) {
            manualLaunchOn = true;
        }
        if (gamepad1.square) {
            manualLaunchOn = false;
        }
        if (gamepad1.left_bumper) {
            launcher.feederReverse();
        } else if (gamepad1.right_bumper) {
            launcher.feederForward();
        } else if (manualLaunchOn) {
            launcher.launch(true);
        } else {
            launcher.launch(false);
        }

        if (gamepad1.cross) {
            intake.startIntake();
        }
        if (gamepad1.circle) {
            intake.stopIntake();
        }
    }

    /** Hold the reset button to declare "I am standing on the default start pose". */
    private void handlePoseReset(double now) {
        boolean held = GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.RESET_POSE_BUTTON);
        boolean idle = controller == null || controller.state() == AutoShootController.State.IDLE;
        if (!held || !idle) {
            resetHeldSinceSec = Double.NaN;
            resetLatched = false;
            return;
        }
        if (Double.isNaN(resetHeldSinceSec)) {
            resetHeldSinceSec = now;
        }
        if (!resetLatched && drive != null && now - resetHeldSinceSec >= BioBuzzConstants.Controls.RESET_POSE_HOLD_SEC) {
            drive.setPose(defaultStartPose());
            poseTrusted = true;
            poseSource = "reset by the driver";
            resetLatched = true;
            gamepad1.rumbleBlips(BioBuzzConstants.Controls.RUMBLE_DONE_BLIPS);
        }
    }

    private void giveFeedback(AutoShootController.Outputs out) {
        switch (out.cue) {
            case STARTED:
                gamepad1.rumble(BioBuzzConstants.Controls.RUMBLE_STARTED_MS);
                break;
            case ARRIVED:
                gamepad1.rumble(BioBuzzConstants.Controls.RUMBLE_ARRIVED_MS);
                break;
            case DONE:
                gamepad1.rumbleBlips(BioBuzzConstants.Controls.RUMBLE_DONE_BLIPS);
                break;
            case FAULT:
                gamepad1.rumble(1.0, 1.0, BioBuzzConstants.Controls.RUMBLE_FAULT_MS);
                break;
            default:
                break;
        }
    }

    private void report(AutoShootController.Outputs out, Pose2 pose) {
        telemetry.addData("Alliance", alliance + "   Drive: " + driveNote);
        if (out == null) {
            telemetry.addLine("AUTO-SHOOT DISABLED: " + driveNote);
        } else {
            telemetry.addData("Auto-shoot", out.message);
            if (out.state != AutoShootController.State.IDLE) {
                telemetry.addData("  shots", out.shotsFired + " / " + BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY
                        + "   target side " + out.targetSide);
                telemetry.addData("  range / aim error", String.format("%.1f in / %.1f deg", out.rangeIn, out.aimErrorDeg));
                telemetry.addData("  flywheel", String.format("%.0f / %.0f tps  feeder=%s", launcher.velocity(),
                        out.flywheelTargetTps, out.feederOn));
            }
            if (out.fault != AutoShootController.Fault.NONE) {
                telemetry.addData("  stopped because", out.fault);
            }
        }
        telemetry.addData("HIVE up side", tracker.upSide() + (tracker.confirmed() ? " (seen)" : " (not confirmed)")
                + "   toggle: " + BioBuzzConstants.Controls.TOGGLE_HIVE_BUTTON);
        telemetry.addData("Vision", lastFrame.status + "  tags=" + lastFrame.tags.size() + "  clusters=" + lastViews.size()
                + (lastViews.isEmpty() ? "" : "  " + lastViews.get(0)));
        if (pose != null) {
            telemetry.addData("Pose", pose + "  [" + poseSource + "]");
        }
    }

    @Override
    public void stop() {
        intake.stopIntake();
        launcher.stop();
        if (vision != null) {
            vision.stop();
        }
        if (drive != null) {
            PoseStore.save(drive.pose(), alliance, tracker.upSide(), tracker.confirmed(), System.nanoTime());
            drive.stop();
        }
    }
}
