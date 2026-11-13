package org.firstinspires.ftc.teamcode.diagnostics;

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CameraModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.ClusterAnalyzer;
import org.firstinspires.ftc.teamcode.autoshoot.core.ClusterView;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.TagObservation;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.GamepadButtons;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.LimelightVision;

/**
 * Shows exactly what the Limelight sees and what the auto-shoot logic makes of it, so every vision constant
 * can be checked with the robot standing still on the field (no driving, no shooting).
 *
 * <p>Place the robot where you tell it it is (the default start pose: center of tile C1 for RED, front against
 * the audience wall) and pick the alliance during INIT. The OpMode then compares the position the camera
 * implies with the position you stated: with the mount constants right they agree within about an inch.
 *
 * <p>Checks to make (see docs/START_HERE.md, "Set up the Limelight camera"):
 * <ul>
 *   <li>tag IDs are 30-45, the family is 36H11C and the pipeline is the AprilTag one;</li>
 *   <li>slide the robot to the LEFT: the left value of the cluster centre must SHRINK (a tag that does not move
 *       ends up farther to the robot's right). If it grows, flip CAMERA_SPACE_CONVENTION;</li>
 *   <li>tx must be positive for a tag on the right of the image (TX_POSITIVE_IS_RIGHT);</li>
 *   <li>the UP/DOWN reading of the cluster must say UP for the cell that is tipped up toward the robot;</li>
 *   <li>the implied position must match the stated pose (fix CAMERA_FORWARD_IN, CAMERA_LEFT_IN,
 *       CAMERA_HEIGHT_IN and CAMERA_PITCH_UP_DEG until it does).</li>
 * </ul>
 *
 * <p><b>Words used here.</b> The <b>Limelight</b> is the smart camera on top of the robot, facing the back with the
 * shooter. It finds <b>AprilTags</b> (square black-and-white codes, each with an ID number) and reports where they are
 * in its image. Each HIVE cell carries four tags, called a <b>cluster</b>. <b>tx</b> and <b>ty</b> are how many degrees
 * a tag is to the right and up from the center of the image. <b>Pipeline</b> is the Limelight's saved detection
 * setup (ours is pipeline 0); <b>Full 3D</b> is its option to also report a tag's distance. A <b>frame</b> is one
 * camera picture's result. A <b>pose</b> is a position (x, y inches) plus a heading. The <b>robot frame</b> is
 * measured from the robot's center of rotation: forward, left and up in inches.
 *
 * <p><b>When to run it and what to do.</b> Use it after mounting or moving the camera, and whenever vision seems off.
 * It is in the Driver Station TeleOp list as "Limelight Diagnostic" (group "AutoShoot"). Put the robot on the stated
 * pose, press INIT, choose the alliance with D-pad left (RED) or right (BLUE) (the buttons come from
 * {@code BioBuzzConstants.Controls}), and watch the "Limelight" status line until it shows the camera running. Press
 * PLAY and read the screen. The robot does not drive or shoot. If you move the robot by hand to test other
 * places, remember that the "stated pose" does not change, so the implied-position error then shows how far you
 * moved it.
 *
 * <p><b>What the screen shows (after PLAY).</b>
 * <ul>
 *   <li>"Frame": the camera status (OK, or why there is no usable data: NOT_CONNECTED, NO_RESULT, STALE, STALLED,
 *       WRONG_PIPELINE), whether a new frame arrived this loop, the image age in milliseconds, the number of frames
 *       seen so far, the number of tags in this frame and how many were thrown away for being too near or too far
 *       (TAG_MIN_RANGE_IN / TAG_MAX_RANGE_IN).</li>
 *   <li>"Raw": each tag's position exactly as the Limelight reports it, in inches, before any conversion.</li>
 *   <li>One line per tag: its ID; tx and ty in degrees; how much of the image it covers (percent); the azimuth (the
 *       direction to the tag in degrees, counter-clockwise from the robot's forward direction); and, if known, its
 *       position in the robot frame (forward, left, up, in inches). "No 3D" means Full 3D is off in the pipeline.</li>
 *   <li>One block per HIVE cell of your alliance that is in view: UP or DOWN (UNKNOWN if the evidence disagrees),
 *       the number of its tags seen, the center of its tag row (forward and left, inches), the range from the ball exit
 *       to the aim point (inches), the turn that would point the shooter at it (degrees, counter-clockwise
 *       positive), and the field position the camera says the robot is at together with how far that is from the
 *       stated pose (dx, dy in inches).</li>
 * </ul>
 */
@TeleOp(name = "Limelight Diagnostic", group = "AutoShoot")
public class LimelightDiagnostic extends OpMode {
    // The "map" of the field (HIVE cells, walls, obstacles).
    private final FieldModel field = new FieldModel();
    // Where the camera is mounted on the robot (from BioBuzzConstants.Vision); turns what the camera sees into
    // positions relative to the robot.
    private final CameraModel camera = CameraModel.fromConstants();
    // Groups the tags by HIVE cell and works out UP / DOWN, range, turn needed and the implied robot position.
    private ClusterAnalyzer analyzer;
    // The Limelight adapter. null if the camera could not be found (then "problem" says why).
    private LimelightVision vision;
    // Which alliance's HIVE to look at. Starts as RED; changed in INIT.
    private Alliance alliance = Alliance.RED;
    // System.nanoTime() (nanoseconds) at PLAY; the clock for the camera code counts from here.
    private long startNanos;
    // Message shown on screen when there is no Limelight.
    private String problem = "";

    // Runs once at INIT: builds the analyzer and starts the camera (selects the AprilTag pipeline and begins polling).
    @Override
    public void init() {
        analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());
        try {
            vision = new LimelightVision(hardwareMap, camera);
            vision.start();
        } catch (RuntimeException e) {
            problem = "No Limelight: " + e.getMessage()
                    + "  (add it in Configure Robot as a USB device named '" + BioBuzzConstants.Hardware.LIMELIGHT + "')";
        }
    }

    // Runs over and over between INIT and PLAY: pick the alliance and watch the camera's own status report.
    @Override
    public void init_loop() {
        if (GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.ALLIANCE_RED_BUTTON)) {
            alliance = Alliance.RED;
        } else if (GamepadButtons.pressed(gamepad1, BioBuzzConstants.Controls.ALLIANCE_BLUE_BUTTON)) {
            alliance = Alliance.BLUE;
        }
        telemetry.addData("Alliance", alliance + "  (" + BioBuzzConstants.Controls.ALLIANCE_RED_BUTTON + " / "
                + BioBuzzConstants.Controls.ALLIANCE_BLUE_BUTTON + ")");
        if (vision != null) {
            // The camera's own status: connected, pipeline number and type, frames per second, temperature, CPU load.
            telemetry.addData("Limelight", vision.statusText());   // blocking call: fine in init_loop
        } else {
            telemetry.addLine(problem);
        }
    }

    // Runs once at PLAY: start the clock.
    @Override
    public void start() {
        startNanos = System.nanoTime();
    }

    // Runs over and over after PLAY: read the camera and show everything it sees.
    @Override
    public void loop() {
        double now = (System.nanoTime() - startNanos) / 1.0e9;
        // The pose we TELL the code the robot has (the default start pose, turned for BLUE if needed). The robot must
        // really be standing there for the comparison further down to mean anything.
        Pose2 stated = alliance.fromRed(Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN,
                BioBuzzConstants.Controls.DEFAULT_START_Y_IN, BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG),
                field.size());
        telemetry.addData("Alliance / stated pose", alliance + "  " + stated + "   (robot must be here)");
        if (vision == null) {
            telemetry.addLine(problem);
            return;
        }

        // This loop's camera snapshot, already converted to inches, degrees and the robot frame.
        VisionFrame frame = vision.read(now);
        telemetry.addData("Frame", frame.status + "  new=" + frame.newFrame + "  age=" + String.format("%.0f ms", frame.ageSec * 1000)
                + "  frames=" + vision.framesSeen() + "  tags=" + frame.tags.size() + "  rejectedRange=" + vision.rejectedForRange());
        if (!vision.lastProblem().isEmpty()) {
            telemetry.addData("Problem", vision.lastProblem());
        }
        telemetry.addData("Raw (camera space, as reported)", vision.rawSummary());

        // One line per tag.
        for (TagObservation t : frame.tags) {
            telemetry.addData("tag " + t.id, String.format("tx=%.1f ty=%.1f area=%.2f azimuth=%.0f deg%s", t.txDeg, t.tyDeg,
                    t.areaPct, Math.toDegrees(t.azimuthRad), t.hasPosition
                            ? String.format("  robot-frame fwd=%.1f left=%.1f up=%.1f in", t.forwardIn, t.leftIn, t.heightIn)
                            : "  (no 3D: enable Full 3D in the pipeline)"));
        }

        // Group the tags into HIVE cells. There is no odometry here, so the stated heading is used.
        List<ClusterView> views = analyzer.analyze(alliance, stated.heading, frame.tags);
        if (views.isEmpty()) {
            telemetry.addLine("No cluster of your alliance in view. Check the pipeline, the pitch and the exposure.");
        }
        for (ClusterView v : views) {
            telemetry.addData("CLUSTER " + v.cell.side, v.orientation + "  tags=" + v.tagCount);
            // hasGeometry: an UP cell whose tags have 3D positions, so range and implied position can be computed.
            if (v.hasGeometry) {
                telemetry.addData("  centre (robot frame)", String.format("fwd=%.1f left=%.1f in", v.centerForwardIn, v.centerLeftIn));
                telemetry.addData("  range to aim point", String.format("%.1f in  (shot table anchor: 37.7)", v.rangeIn));
                telemetry.addData("  turn to point the shooter", String.format("%.1f deg", Math.toDegrees(v.aimErrorRad)));
                // Where the camera says the robot is, and how far that is from where we said it is. Small = mount
                // constants are right.
                telemetry.addData("  IMPLIED robot position", String.format("(%.1f, %.1f)  error vs stated: dx=%.1f dy=%.1f in",
                        v.impliedX, v.impliedY, v.impliedX - stated.x, v.impliedY - stated.y));
            }
        }
    }

    // Runs once at STOP: stop polling the camera.
    @Override
    public void stop() {
        if (vision != null) {
            vision.stop();
        }
    }
}
