package org.firstinspires.ftc.teamcode.autoshoot.hardware;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.CameraModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.TagObservation;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;

/**
 * The only class that knows about the Limelight SDK. Once per loop it turns the latest result into an
 * immutable {@link VisionFrame} in inches, degrees and the robot frame; everything else works from that.
 *
 * <p>Facts about the SDK this class is written around (checked against the 12.0.0 jars):
 * <ul>
 *   <li>The Control Hub polls the camera on a background thread; {@code getLatestResult()} returns the
 *       same object until the next poll, so a NEW frame is recognised by a changed {@code getTimestamp()}.</li>
 *   <li>{@code getStaleness()} is the time since the hub last parsed a reply, not the age of the image;
 *       the image age is that plus the camera's capture and targeting latency.</li>
 *   <li>{@code pipelineSwitch}, {@code getStatus} and friends are blocking HTTP calls, so they are used
 *       only in {@link #start} and {@link #statusText}, never in the loop.</li>
 *   <li>Poses are in METERS and DEGREES; a missing pose is an all-zero pose, never null.</li>
 *   <li>A result with no tags in view is "not valid" but is still a healthy answer.</li>
 * </ul>
 */
public final class LimelightVision {
    private static final double METERS_TO_INCHES = 39.37007874;

    private final Limelight3A limelight;
    private final CameraModel camera;

    private double lastStamp = Double.NaN;
    private double lastNewFrameAtSec = Double.NaN;
    private int rejectedRange;
    private int framesSeen;
    private String lastProblem = "";
    private final StringBuilder rawSummary = new StringBuilder();

    /** @throws IllegalArgumentException if the Limelight is not in the robot configuration */
    public LimelightVision(HardwareMap hardwareMap, CameraModel camera) {
        this.limelight = hardwareMap.get(Limelight3A.class, BioBuzzConstants.Hardware.LIMELIGHT);
        this.camera = camera;
    }

    /** Selects the AprilTag pipeline and starts polling. Blocking HTTP: call from init/start, not from the loop. */
    public void start() {
        limelight.setPollRateHz(BioBuzzConstants.Vision.POLL_RATE_HZ);
        limelight.pipelineSwitch(BioBuzzConstants.Vision.PIPELINE_INDEX);
        limelight.start();
    }

    public void stop() {
        try {
            limelight.stop();
        } catch (RuntimeException ignored) {
            // shutting down; nothing useful to do
        }
    }

    public boolean connected() {
        return limelight.isConnected();
    }

    /** Builds this loop's snapshot. Never throws; a problem becomes a status. */
    public VisionFrame read(double nowSec) {
        try {
            return readInternal(nowSec);
        } catch (RuntimeException e) {
            lastProblem = e.getClass().getSimpleName() + ": " + e.getMessage();
            return VisionFrame.unavailable(VisionFrame.Status.NO_RESULT, nowSec);
        }
    }

    private VisionFrame readInternal(double nowSec) {
        if (!limelight.isConnected()) {
            return VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, nowSec);
        }
        LLResult r = limelight.getLatestResult();
        if (r == null) {
            return VisionFrame.unavailable(VisionFrame.Status.NO_RESULT, nowSec);
        }

        double stamp = r.getTimestamp();
        if (stamp == 0.0) {
            return VisionFrame.unavailable(VisionFrame.Status.NO_RESULT, nowSec);     // nothing received yet
        }
        boolean newFrame = Double.isNaN(lastStamp) || stamp != lastStamp;
        if (newFrame) {
            lastStamp = stamp;
            lastNewFrameAtSec = nowSec;
            framesSeen++;
        } else if (nowSec - lastNewFrameAtSec > BioBuzzConstants.Vision.FRAME_STALL_MS / 1000.0) {
            return VisionFrame.unavailable(VisionFrame.Status.STALLED, nowSec);
        }

        if (r.getPipelineIndex() != BioBuzzConstants.Vision.PIPELINE_INDEX) {
            return VisionFrame.unavailable(VisionFrame.Status.WRONG_PIPELINE, nowSec);
        }

        double ageMs = r.getStaleness() + r.getCaptureLatency() + r.getTargetingLatency();
        if (ageMs > BioBuzzConstants.Vision.MAX_FRAME_AGE_MS) {
            return VisionFrame.unavailable(VisionFrame.Status.STALE, nowSec);
        }

        List<TagObservation> tags = new ArrayList<TagObservation>();
        rawSummary.setLength(0);
        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            TagObservation t = convert(f);
            if (t != null) {
                tags.add(t);
            }
        }
        return new VisionFrame(VisionFrame.Status.OK, newFrame, ageMs / 1000.0, nowSec, tags);
    }

    private TagObservation convert(LLResultTypes.FiducialResult f) {
        double tx = f.getTargetXDegrees();
        double ty = f.getTargetYDegrees();
        if (!BioBuzzConstants.Vision.TX_POSITIVE_IS_RIGHT) {
            tx = -tx;
        }
        if (!BioBuzzConstants.Vision.TY_POSITIVE_IS_UP) {
            ty = -ty;
        }
        double azimuth = camera.azimuthOf(tx, ty);
        double area = f.getTargetArea();

        Pose3D pose = f.getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null) {
            return TagObservation.bearingOnly(f.getFiducialId(), tx, ty, area, azimuth);
        }
        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        rawSummary.append(String.format("id%d raw=(%.1f, %.1f, %.1f) in; ", f.getFiducialId(), p.x, p.y, p.z));
        if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) {
            return TagObservation.bearingOnly(f.getFiducialId(), tx, ty, area, azimuth);   // "Full 3D" is off
        }

        double right;
        double down;
        double forward;
        if (BioBuzzConstants.Vision.CAMERA_SPACE_CONVENTION
                == BioBuzzConstants.Vision.CameraSpaceConvention.OPENCV_RIGHT_DOWN_FORWARD) {
            right = p.x;
            down = p.y;
            forward = p.z;
        } else {
            right = -p.y;         // x forward, y left, z up
            down = -p.z;
            forward = p.x;
        }
        double distance = Math.sqrt(right * right + down * down + forward * forward);
        if (distance < BioBuzzConstants.Vision.TAG_MIN_RANGE_IN || distance > BioBuzzConstants.Vision.TAG_MAX_RANGE_IN) {
            rejectedRange++;
            return null;
        }
        double[] robot = camera.cameraToRobot(right, down, forward);
        return TagObservation.withPosition(f.getFiducialId(), tx, ty, area, azimuth, robot[0], robot[1], robot[2]);
    }

    // ------------------------------------------------------------------------------ diagnostics

    public int framesSeen() {
        return framesSeen;
    }

    public int rejectedForRange() {
        return rejectedRange;
    }

    public String lastProblem() {
        return lastProblem;
    }

    /**
     * The camera-space positions exactly as the Limelight reported them for the last frame (inches, before any
     * axis conversion), for the diagnostic OpMode.
     */
    public String rawSummary() {
        return rawSummary.toString();
    }

    /**
     * Human-readable device status. This is a BLOCKING HTTP call: use it only in init_loop or a
     * diagnostic OpMode.
     */
    public String statusText() {
        try {
            LLStatus s = limelight.getStatus();
            return String.format("connected=%s pipeline=%d type=%s fps=%.0f temp=%.0fC cpu=%.0f%%",
                    limelight.isConnected(), s.getPipelineIndex(), s.getPipelineType(), s.getFps(), s.getTemp(),
                    s.getCpu());
        } catch (RuntimeException e) {
            return "status unavailable: " + e.getMessage();
        }
    }
}
