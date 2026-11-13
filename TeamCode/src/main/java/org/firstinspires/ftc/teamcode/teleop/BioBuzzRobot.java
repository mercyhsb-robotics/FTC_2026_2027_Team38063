package org.firstinspires.ftc.teamcode.teleop;

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
import org.firstinspires.ftc.teamcode.autoshoot.core.PoseCorrector;
import org.firstinspires.ftc.teamcode.autoshoot.core.PoseStore;
import org.firstinspires.ftc.teamcode.autoshoot.core.ScoringPlanner;
import org.firstinspires.ftc.teamcode.autoshoot.core.ShotTable;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.GamepadButtons;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.LimelightVision;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.PedroDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.mecanumdrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The driver-controlled OpMode (TeleOp): the program the drivers run for the driver-controlled part of a
 * match, and the one to use for practice driving.
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
 *
 * <p><b>Words used in this file.</b>
 * <ul>
 *   <li><b>OpMode</b>: one program the Driver Station can run. This one is an "iterative" OpMode: the robot
 *       controller calls {@code init()} once when you press INIT, {@code init_loop()} over and over while you
 *       wait, {@code start()} once when you press PLAY, {@code loop()} over and over during the run (many times
 *       a second) and {@code stop()} once at the end.</li>
 *   <li><b>HIVE</b>: each alliance's scoring structure. It has two CELLs (openings), one toward the audience wall
 *       and one toward the far "rear" wall; exactly one is tipped UP and can be scored into. <b>POLLEN</b>: the
 *       balls the robot shoots (it may hold at most 4).</li>
 *   <li><b>Pinpoint</b>: the goBILDA odometry computer. Two small wheels ("pods") on the floor measure how far the
 *       robot has moved and the Pinpoint also tracks its heading, so the robot always knows its position on the
 *       field. <b>Pedro Pathing</b>: the library that drives the robot to a point or along a path using that
 *       position. Its main object is the <b>Follower</b>.</li>
 *   <li><b>Limelight</b>: the smart camera that finds <b>AprilTags</b> (square black-and-white codes) on the HIVE.
 *       <b>Pose</b>: where the robot is (x and y in inches) and which way it faces (heading).</li>
 *   <li><b>Mecanum</b>: wheels with angled rollers that let the robot drive in any direction without turning;
 *       moving sideways is called <b>strafing</b>.</li>
 *   <li><b>Flywheel</b>: the fast-spinning wheel at the back that throws a POLLEN out; <b>feeder</b>: the servo that
 *       pushes the next POLLEN into it. The flywheel's speed is measured in encoder <b>ticks</b> per second ("tps";
 *       a tick is one count of the motor's encoder).</li>
 * </ul>
 *
 * <p><b>When to run it.</b> Pick "BioBuzzRobot" in the Driver Station TeleOp list (the annotation has no name, so
 * the class name is shown), press INIT, choose the alliance with D-pad left / right (the gamepad light turns red or
 * blue), put the robot on the start pose, then press PLAY. The start pose is the one saved by an earlier run of
 * this OpMode if that run ended less than 5 minutes ago (BioBuzzConstants.Controls.POSE_CARRYOVER_MAX_AGE_SEC) and
 * had the same alliance, otherwise the default: the center of tile C1 with the robot's FRONT against the audience
 * wall (see BioBuzzConstants.Controls.DEFAULT_START_*). The autonomous OpModes do not save a pose yet, so after
 * an autonomous routine the robot must be put on the default start pose by hand and the pose reset button held
 * (below).
 *
 * <p><b>What you should see.</b> During INIT the Driver Station shows the alliance, which drive is in use
 * ("Pedro + Pinpoint" is the good one), any constants warnings, the placeholder values that still need measuring,
 * the Limelight status and where the start pose comes from. While driving it shows the auto-shoot state, shot count,
 * range, aim error, flywheel speed, which HIVE cell the robot thinks is up, what the camera sees and the pose.
 * The gamepad rumbles when an auto-shoot starts, arrives, finishes (two blips) or gives up (long, strong).
 *
 * <p><b>Controls (gamepad 1).</b> The shoot, "other cell is up", pose reset and alliance buttons are assigned in
 * {@code BioBuzzConstants.Controls}; the sticks' sign, scale and deadband are there too. The manual mechanism
 * buttons (cross, circle, triangle, square, bumpers, triggers) are fixed in {@link #manualControls()}.
 * <ul>
 *   <li>Left stick: drive forward / back and strafe left / right, relative to the robot. Right stick, left / right:
 *       turn. Pushing the left stick hard while the robot is driving itself cancels the automatic drive.</li>
 *   <li>D-pad up (hold): drive to the scoring pose and shoot everything; release to take over again.</li>
 *   <li>D-pad down: "the other cell is up": flips which cell the robot believes is tipped up.</li>
 *   <li>Share (hold about 0.6 s, only while not auto-shooting): "I am standing on the default start pose":
 *       resets the odometry position to it.</li>
 *   <li>D-pad left / right (INIT only): choose the RED / BLUE alliance.</li>
 *   <li>Cross / circle: intake motor on / off.</li>
 *   <li>Left / right trigger (hold): the left / right corner-pulling intake servo.</li>
 *   <li>Triangle / square: manual launch on / off (the feeder runs once the flywheel is fast enough).</li>
 *   <li>Left / right bumper (hold): feeder runs backward / forward, for clearing a jam.</li>
 * </ul>
 * While the robot is shooting automatically the intake, the corner servos and the manual launch buttons are
 * switched off (the robot may hold at most 4 POLLEN), so only the sticks and the buttons above work.
 */
@TeleOp
public class BioBuzzRobot extends OpMode {
    // ---- mechanisms
    // The shooter: the flywheel and the feeder servo that pushes a POLLEN into it.
    private Launcher launcher;
    // The front intake roller motor.
    private Intake intake;
    // The two corner-pulling intake servos. A CRServo is a continuous-rotation servo: it spins like a motor, and
    // setPower(-1..1) sets its speed and direction instead of a position.
    private CRServo servoLeft;
    private CRServo servoRight;

    // ---- drive: Pedro owns the motors when it starts; otherwise the legacy class does
    // The Pedro/Pinpoint drive. null if Pedro could not start (then auto-shoot is off).
    private PedroDrive drive;
    // The simple fallback drive (no odometry). Only created if Pedro failed.
    private MecanumDrive legacyDrive;
    // Text for the Driver Station: which drive is in use, or why the fallback was chosen.
    private String driveNote = "";

    // ---- vision
    // The Limelight camera. null if it could not be found.
    private LimelightVision vision;
    // Text for the Driver Station: whether the camera started.
    private String visionNote = "";

    // ---- logic
    // The "map" of the field: HIVE cells, walls and obstacles.
    private final FieldModel field = new FieldModel();
    // The shot table: distance to the HIVE -> flywheel speed in ticks per second.
    private ShotTable table;
    // What the constants check found at INIT: errors, warnings and the values still marked ADD (placeholders).
    private ConstantsCheck.Report constantsReport;
    // Where the camera is mounted on the robot (from BioBuzzConstants.Vision); needed to turn what it sees into
    // robot-relative positions.
    private final CameraModel camera = CameraModel.fromConstants();
    // Turns the AprilTags in one camera frame into facts about each HIVE cell (up or down, range, turn needed).
    private ClusterAnalyzer analyzer;
    // Picks the closest legal place to stand and shoot from.
    private ScoringPlanner planner;
    // Which alliance we are. Starts as RED, can be changed in INIT.
    private Alliance alliance = Alliance.RED;
    // The robot's belief about which cell of our HIVE is tipped up.
    private HiveTracker tracker;
    // The hold-to-shoot sequence (plan, drive, align, feed). A state machine: it is always in exactly one state.
    private AutoShootController controller;
    // Uses the camera to correct slow odometry drift (off unless BioBuzzConstants.Vision.POSE_CORRECTION_ENABLED).
    private PoseCorrector corrector;

    // ---- state
    // System.nanoTime() (nanoseconds) when PLAY was pressed; nowSec() counts from here.
    private long startNanos;
    // True once the odometry has been given a start pose we believe. Auto-shoot refuses to run without it.
    private boolean poseTrusted;
    // Text for the Driver Station: where the current pose came from.
    private String poseSource = "";
    // Was the toggle button down on the previous loop? Used to notice a new press rather than a held button.
    private boolean prevToggle;
    // Time (seconds, nowSec clock) when the pose reset button started being held; NaN = not being held.
    private double resetHeldSinceSec = Double.NaN;
    // True once the pose reset has fired for the current hold, so it happens once per press.
    private boolean resetLatched;
    // The auto-shoot state on the previous loop; used to notice the moment the shoot button is released.
    private AutoShootController.State prevState = AutoShootController.State.IDLE;
    // The triangle / square latch: true while the manual launch is switched on.
    private boolean manualLaunchOn;
    // The latest auto-shoot output. Only stored; nothing reads it yet.
    private AutoShootController.Outputs lastOut;
    // The latest camera results, kept so report() can show them.
    private List<ClusterView> lastViews = Collections.emptyList();
    private VisionFrame lastFrame;

    /** Runs once when INIT is pressed: checks the constants, finds the hardware and builds the logic objects. */
    @Override
    public void init() {
        // Refuse nonsense in BioBuzzConstants here, with a message that names the problem, instead of failing later.
        constantsReport = ConstantsCheck.run();
        if (!constantsReport.ok()) {
            throw new IllegalStateException("BioBuzzConstants has errors: " + constantsReport.errors);
        }
        table = ShotTable.fromConstants();

        // Bulk caching: instead of one slow request to the hub for every encoder or sensor reading, the hub sends
        // them all in one batch and the SDK reuses that batch. AUTO means the SDK refreshes the batch by itself.
        // (A LynxModule is a Control Hub or Expansion Hub.)
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
        // (Constants.create builds Pedro's Follower from the settings in pedro/Constants.java.)
        try {
            drive = new PedroDrive(Constants.create(hardwareMap));
            driveNote = "Pedro + Pinpoint";
        } catch (RuntimeException e) {
            drive = null;
            driveNote = "LEGACY drive (Pedro/Pinpoint failed: " + e.getMessage() + ")";
            legacyDrive = new MecanumDrive();
            legacyDrive.init(hardwareMap);
        }

        // The camera is optional: without it the robot still drives and shoots, but cannot confirm which cell is up.
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
        corrector = new PoseCorrector(PoseCorrector.Config.fromConstants());

        // If a recent OpMode (normally this one, run a moment ago) saved its pose, start with the same alliance.
        PoseStore.Snapshot snap = PoseStore.load(System.nanoTime(), BioBuzzConstants.Controls.POSE_CARRYOVER_MAX_AGE_SEC);
        if (snap != null) {
            alliance = snap.alliance;
        }
        showAllianceColor();
    }

    /** Runs over and over after INIT until PLAY: lets the driver pick the alliance and shows the pre-match checks. */
    @Override
    public void init_loop() {
        // Only change when the other alliance is chosen, so the light is not rewritten on every loop.
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
        // frames = new camera frames received so far; link = whether the Control Hub can reach the camera.
        telemetry.addData("Vision", visionNote + (vision != null ? "  frames=" + vision.framesSeen() + " link=" + vision.connected() : ""));
        PoseStore.Snapshot snap = PoseStore.load(System.nanoTime(), BioBuzzConstants.Controls.POSE_CARRYOVER_MAX_AGE_SEC);
        telemetry.addData("Start pose", snap != null && snap.alliance == alliance
                ? "carried over from the previous OpMode" : "default: center of tile C1, front against the audience wall");
    }

    /** Runs once when PLAY is pressed: starts the clock, builds the alliance-specific logic and sets the start pose. */
    @Override
    public void start() {
        startNanos = System.nanoTime();
        // These need the alliance, which is only final now that INIT is over.
        tracker = new HiveTracker(field, alliance, HiveTracker.Config.fromConstants());
        controller = new AutoShootController(alliance, field, planner, table, tracker,
                AutoShootController.Config.fromConstants());

        PoseStore.Snapshot snap = PoseStore.load(System.nanoTime(), BioBuzzConstants.Controls.POSE_CARRYOVER_MAX_AGE_SEC);
        Pose2 startPose;
        if (snap != null && snap.alliance == alliance) {
            // Same alliance as the saved run: also keep its belief about which HIVE cell is up.
            startPose = snap.pose;
            tracker.reset(snap.hiveUpSide, snap.hiveConfirmed);
            poseSource = "carried over from the previous OpMode";
        } else {
            startPose = defaultStartPose();
            poseSource = "default start pose (tile C1, front on the audience wall)";
        }
        // Tell the odometry where the robot is. It cannot know by itself, so the robot must really be standing there.
        if (drive != null) {
            drive.setPose(startPose);
            poseTrusted = true;
        }
    }

    /**
     * The default start pose for the chosen alliance. The constants are written for RED; the BLUE pose is the same
     * point turned half a turn about the field center.
     */
    private Pose2 defaultStartPose() {
        Pose2 red = Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN,
                BioBuzzConstants.Controls.DEFAULT_START_Y_IN, BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG);
        return alliance.fromRed(red, field.size());
    }

    /** Lights the gamepad's light bar in the alliance color so the driver can see the choice at a glance. */
    private void showAllianceColor() {
        double[] c = alliance == Alliance.RED ? BioBuzzConstants.Controls.RED_LED : BioBuzzConstants.Controls.BLUE_LED;
        try {
            gamepad1.setLedColor(c[0], c[1], c[2], Gamepad.LED_DURATION_CONTINUOUS);
        } catch (RuntimeException ignored) {
            // not every controller has a light
        }
    }

    /** Seconds since PLAY was pressed. This is the clock the auto-shoot logic uses. */
    private double nowSec() {
        return (System.nanoTime() - startNanos) / 1.0e9;
    }

    // ---------------------------------------------------------------------------------------------

    /**
     * Runs over and over after PLAY, many times a second. Every pass does the same five things in order: sense,
     * read the driver, decide, act, report.
     */
    @Override
    public void loop() {
        double now = nowSec();

        // 1. Sense.
        // The drive update must come first, once per loop: it reads the odometry and sends power to the motors.
        if (drive != null) {
            drive.update();
        }
        // The robot's current pose (inches, radians), or null if there is no Pedro drive.
        Pose2 pose = drive != null ? drive.pose() : null;
        // This loop's camera snapshot. Without a Limelight an "unavailable" frame is used, so the code below
        // does not need special cases.
        VisionFrame frame = vision != null ? vision.read(now)
                : VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, now);
        // What the camera says about each HIVE cell of our alliance (uses the heading from odometry).
        List<ClusterView> views = pose != null ? analyzer.analyze(alliance, pose.heading, frame.tags)
                : Collections.<ClusterView>emptyList();
        if (pose != null) {
            // Update the belief about which cell is up, given what the camera sees and which end of the HIVE we are on.
            tracker.update(now, frame, views, HiveTracker.sideOf(pose, field));
            pose = correctDrift(now, pose, frame, views);
        }
        lastFrame = frame;
        lastViews = views;

        // 2. Driver inputs.
        double[] sticks = readSticks();
        boolean toggle = GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.TOGGLE_HIVE_BUTTON);
        // Act only on the moment the button goes down (a rising edge), not on every loop it is held.
        if (toggle && !prevToggle) {
            tracker.toggle();
            gamepad1.rumble(BioBuzzConstants.Controls.RUMBLE_STARTED_MS);
        }
        prevToggle = toggle;
        boolean shootHeld = GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.SHOOT_BUTTON);
        handlePoseReset(now);

        // 3. Decide. Hand every measurement to the auto-shoot state machine and get back what to command.
        // Without Pedro there is no pose, so there is no auto-shoot and out stays null.
        AutoShootController.Outputs out = null;
        if (drive != null) {
            AutoShootController.Inputs in = new AutoShootController.Inputs();
            in.nowSec = now;                             // seconds since PLAY
            in.shootHeld = shootHeld;
            in.pose = pose;
            in.speedInS = drive.speedInPerSec();         // inches per second
            in.omegaRadS = drive.omegaRadPerSec();       // radians per second
            in.poseTrusted = poseTrusted;
            in.flywheelTps = launcher.velocity();        // flywheel speed, encoder ticks per second
            in.frame = frame;
            in.views = views;
            in.stickMagnitude = sticks[3];               // 0..1, lets the controller see the driver taking over
            out = controller.update(in);
            lastOut = out;
        }

        // 4. Act.
        // "Automatic" covers every auto-shoot state except IDLE, including DONE and FAULT, which last until the
        // shoot button is released.
        boolean automatic = out != null && out.state != AutoShootController.State.IDLE;
        if (automatic) {
            intake.stopIntake();                    // never intake while shooting: the robot may hold at most 4
            servoLeft.setPower(0);
            servoRight.setPower(0);
            launcher.setTargetVelocity(out.flywheelTargetTps);   // ticks per second
            launcher.setFeederOn(out.feederOn);                  // the controller decides when a ball may be fed
            // Carry out the controller's drive command: follow a route, hold a pose, or (FAULT) use the sticks.
            drive.apply(out, sticks[0], sticks[1], sticks[2]);
            manualLaunchOn = false;
        } else {
            if (prevState != AutoShootController.State.IDLE) {
                launcher.stop();                    // the button was just released: stop the flywheel and feeder
            }
            if (drive != null) {
                drive.apply(out, sticks[0], sticks[1], sticks[2]);
            } else {
                // Legacy drive wants the raw stick values; see MecanumDrive.drive for the sign convention.
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

    /**
     * Lets a good sighting of the HIVE tags nudge the odometry position back toward the truth (see
     * {@link PoseCorrector}). Skipped while a ball is being fed, so the robot never shifts in the middle of a shot.
     *
     * @return the pose to use for the rest of this loop (the corrected one if the position was moved)
     */
    private Pose2 correctDrift(double now, Pose2 pose, VisionFrame frame, List<ClusterView> views) {
        if (controller != null && controller.state() == AutoShootController.State.FEED) {
            return pose;
        }
        PoseCorrector.Correction fix = corrector.update(now, pose, drive.speedInPerSec(), drive.omegaRadPerSec(),
                frame, views);
        if (fix == null) {
            return pose;                   // no correction this loop (off, robot moving, bad sighting, ...)
        }
        Pose2 corrected = new Pose2(pose.x + fix.dx, pose.y + fix.dy, pose.heading);   // heading stays with the gyro
        drive.setPose(corrected);
        return corrected;
    }

    /**
     * Reads the sticks and converts them to the robot's drive convention.
     *
     * @return {forward, strafe, turn, translation stick magnitude}: forward / strafe / turn are ready for the drive
     *         (each -1..1: forward positive, strafe positive to the robot's LEFT, turn positive counter-clockwise),
     *         after the deadband and the sign and scale constants in BioBuzzConstants.Controls. The fourth value
     *         is how far the left stick is pushed, 0..1.
     */
    private double[] readSticks() {
        double lx = -gamepad1.left_stick_x;       // stick right = move right = negative strafe (+strafe is LEFT)
        double ly = -gamepad1.left_stick_y;       // stick up is negative on the SDK; forward is positive
        double rx = -gamepad1.right_stick_x;      // stick right = clockwise = negative turn (+turn is counter-clockwise)
        double mag = Math.hypot(lx, ly);
        double db = BioBuzzConstants.Controls.STICK_DEADBAND;
        // Deadband: tiny stick values (a stick that does not rest exactly at zero) count as zero. Above it the
        // remaining range is stretched back to 0..1 so the robot does not jump when the stick leaves the deadband.
        double scale = mag < db ? 0.0 : (mag - db) / (1.0 - db) / mag;
        double fwd = ly * scale * BioBuzzConstants.Controls.MANUAL_FORWARD_SIGN * BioBuzzConstants.Controls.MANUAL_DRIVE_SCALE;
        double str = lx * scale * BioBuzzConstants.Controls.MANUAL_STRAFE_SIGN * BioBuzzConstants.Controls.MANUAL_DRIVE_SCALE;
        double turn = Math.abs(rx) < db ? 0.0 : rx * BioBuzzConstants.Controls.MANUAL_TURN_SIGN * BioBuzzConstants.Controls.MANUAL_TURN_SCALE;
        return new double[]{fwd, str, turn, Math.min(1.0, mag)};
    }

    /**
     * The existing manual launcher, intake and corner-servo controls. Only used when the robot is not shooting
     * automatically. These buttons are fixed here, not in BioBuzzConstants.Controls.
     */
    private void manualControls() {
        // Corner servos on the triggers. The left one runs at power -1 and the right one at +1 (so they turn in
        // opposite directions). Each runs only while its trigger is pulled past the threshold.
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

        // The intake motor stays on after cross until circle is pressed.
        if (gamepad1.cross) {
            intake.startIntake();
        }
        if (gamepad1.circle) {
            intake.stopIntake();
        }
    }

    /**
     * Hold the reset button to declare "I am standing on the default start pose". It needs a hold of
     * BioBuzzConstants.Controls.RESET_POSE_HOLD_SEC so it cannot happen by accident, works only while auto-shoot
     * is idle, and fires once per hold.
     */
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

    /**
     * Turns the controller's one-shot cue into gamepad rumble (the driver may be looking at the field, not the
     * screen). The durations are in BioBuzzConstants.Controls.
     */
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
                // Both rumble motors at full strength (1.0 = 100 %) so a fault is hard to miss.
                gamepad1.rumble(1.0, 1.0, BioBuzzConstants.Controls.RUMBLE_FAULT_MS);
                break;
            default:
                break;
        }
    }

    /** Puts the useful numbers on the Driver Station screen (docs/TUNING.md explains the Waiting / Stopped reasons). */
    private void report(AutoShootController.Outputs out, Pose2 pose) {
        telemetry.addData("Alliance", alliance + "   Drive: " + driveNote);
        if (out == null) {
            telemetry.addLine("AUTO-SHOOT DISABLED: " + driveNote);
        } else {
            // The state in words: "Ready", "Driving to the scoring pose", "Waiting: <reason>", "Stopped: <fault>", ...
            telemetry.addData("Auto-shoot", out.message);
            if (out.state != AutoShootController.State.IDLE) {
                telemetry.addData("  shots", out.shotsFired + " / " + BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY
                        + "   target side " + out.targetSide);
                // range: inches from the ball exit to the aim point; aim error: degrees the robot still has to turn.
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
            telemetry.addData("Drift correction", corrector.status());
        }
    }

    /** Runs once at STOP: switches the mechanisms off, stops the camera and saves the pose for the next OpMode. */
    @Override
    public void stop() {
        intake.stopIntake();
        launcher.stop();
        if (vision != null) {
            vision.stop();
        }
        if (drive != null) {
            // Save where we ended, so the next OpMode (for example a restarted TeleOp) can start from here. If STOP
            // was pressed before START there is no HIVE belief yet (the tracker is created in start()), so skip it.
            if (tracker != null) {
                PoseStore.save(drive.pose(), alliance, tracker.upSide(), tracker.confirmed(), System.nanoTime());
            }
            drive.stop();
        }
    }
}
