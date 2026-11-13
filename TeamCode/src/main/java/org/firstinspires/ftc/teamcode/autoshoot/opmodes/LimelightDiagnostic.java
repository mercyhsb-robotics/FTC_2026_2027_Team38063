package org.firstinspires.ftc.teamcode.autoshoot.opmodes;

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
 * <p>Checks to make (see docs/LIMELIGHT_SETUP.md):
 * <ul>
 *   <li>tag IDs are 30-45, the family is 36H11C and the pipeline is the AprilTag one;</li>
 *   <li>slide the robot to the LEFT: the left value of the cluster centre must grow. If not, flip
 *       CAMERA_SPACE_CONVENTION;</li>
 *   <li>tx must be positive for a tag on the right of the image (TX_POSITIVE_IS_RIGHT);</li>
 *   <li>the UP/DOWN reading of the cluster must say UP for the cell that is tipped up toward the robot;</li>
 *   <li>the implied position must match the stated pose (fix CAMERA_FORWARD_IN, CAMERA_LEFT_IN,
 *       CAMERA_HEIGHT_IN and CAMERA_PITCH_UP_DEG until it does).</li>
 * </ul>
 */
@TeleOp(name = "Limelight Diagnostic", group = "AutoShoot")
public class LimelightDiagnostic extends OpMode {
    private final FieldModel field = new FieldModel();
    private final CameraModel camera = CameraModel.fromConstants();
    private ClusterAnalyzer analyzer;
    private LimelightVision vision;
    private Alliance alliance = Alliance.RED;
    private long startNanos;
    private String problem = "";

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
            telemetry.addData("Limelight", vision.statusText());   // blocking call: fine in init_loop
        } else {
            telemetry.addLine(problem);
        }
    }

    @Override
    public void start() {
        startNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        double now = (System.nanoTime() - startNanos) / 1.0e9;
        Pose2 stated = alliance.fromRed(Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN,
                BioBuzzConstants.Controls.DEFAULT_START_Y_IN, BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG),
                field.size());
        telemetry.addData("Alliance / stated pose", alliance + "  " + stated + "   (robot must be here)");
        if (vision == null) {
            telemetry.addLine(problem);
            return;
        }

        VisionFrame frame = vision.read(now);
        telemetry.addData("Frame", frame.status + "  new=" + frame.newFrame + "  age=" + String.format("%.0f ms", frame.ageSec * 1000)
                + "  frames=" + vision.framesSeen() + "  tags=" + frame.tags.size() + "  rejectedRange=" + vision.rejectedForRange());
        if (!vision.lastProblem().isEmpty()) {
            telemetry.addData("Problem", vision.lastProblem());
        }
        telemetry.addData("Raw (camera space, as reported)", vision.rawSummary());

        for (TagObservation t : frame.tags) {
            telemetry.addData("tag " + t.id, String.format("tx=%.1f ty=%.1f area=%.2f azimuth=%.0f deg%s", t.txDeg, t.tyDeg,
                    t.areaPct, Math.toDegrees(t.azimuthRad), t.hasPosition
                            ? String.format("  robot-frame fwd=%.1f left=%.1f up=%.1f in", t.forwardIn, t.leftIn, t.heightIn)
                            : "  (no 3D: enable Full 3D in the pipeline)"));
        }

        List<ClusterView> views = analyzer.analyze(alliance, stated.heading, frame.tags);
        if (views.isEmpty()) {
            telemetry.addLine("No cluster of your alliance in view. Check the pipeline, the pitch and the exposure.");
        }
        for (ClusterView v : views) {
            telemetry.addData("CLUSTER " + v.cell.side, v.orientation + "  tags=" + v.tagCount);
            if (v.hasGeometry) {
                telemetry.addData("  centre (robot frame)", String.format("fwd=%.1f left=%.1f in", v.centerForwardIn, v.centerLeftIn));
                telemetry.addData("  range to aim point", String.format("%.1f in  (shot table anchor: 37.7)", v.rangeIn));
                telemetry.addData("  turn to point the shooter", String.format("%.1f deg", Math.toDegrees(v.aimErrorRad)));
                telemetry.addData("  IMPLIED robot position", String.format("(%.1f, %.1f)  error vs stated: dx=%.1f dy=%.1f in",
                        v.impliedX, v.impliedY, v.impliedX - stated.x, v.impliedY - stated.y));
            }
        }
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.stop();
        }
    }
}
