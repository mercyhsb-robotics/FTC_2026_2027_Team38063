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
 * <p>What the Limelight is: a small camera with its own computer. It finds the AprilTags itself, using a saved
 * set of vision settings called a "pipeline" (ours is pipeline 0, set up as described in
 * docs/LIMELIGHT_SETUP.md), and the Control Hub only reads the answers over USB. So this class never looks at
 * pictures; it reads a list of "this tag ID is at this angle, and this far away".
 *
 * <p>Where it sits in the data flow: Limelight camera -> this class -> {@link VisionFrame} (a list of
 * {@link TagObservation}) -> {@code ClusterAnalyzer} -> {@code HiveTracker} and the rest. {@code BioBuzzRobot}
 * (TeleOp) builds it in {@code init()}, calls {@link #start} once, then {@link #read} every loop and
 * {@link #stop} at the end; the Limelight Diagnostic OpMode uses it too. Nothing else in the project should
 * import a {@code com.qualcomm.hardware.limelightvision} class: if the Limelight SDK changes, this is the one
 * file to fix.
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
    // Not used at the moment: Position.toUnit(DistanceUnit.INCH) in convert() does the meters-to-inches conversion.
    private static final double METERS_TO_INCHES = 39.37007874;

    private final Limelight3A limelight;
    private final CameraModel camera;

    // The timestamp of the last result we looked at (NaN before the first). A different timestamp = a new frame.
    private double lastStamp = Double.NaN;
    // The logic-clock time (seconds) at which we last saw a NEW frame; used to spot a frozen camera.
    private double lastNewFrameAtSec = Double.NaN;
    // Diagnostics for the telemetry screens: tags dropped for being too near/far, and how many frames we have seen.
    private int rejectedRange;
    private int framesSeen;
    private String lastProblem = "";
    // The raw 3D numbers the Limelight gave for the last frame, kept as text for the diagnostic OpMode.
    private final StringBuilder rawSummary = new StringBuilder();

    /**
     * @param camera where the lens is mounted on the robot, used to convert tag positions to the robot frame
     * @throws IllegalArgumentException if the Limelight is not in the robot configuration
     */
    public LimelightVision(HardwareMap hardwareMap, CameraModel camera) {
        this.limelight = hardwareMap.get(Limelight3A.class, BioBuzzConstants.Hardware.LIMELIGHT);
        this.camera = camera;
    }

    /** Selects the AprilTag pipeline and starts polling. Blocking HTTP: call from init/start, not from the loop. */
    public void start() {
        // How many times per second the Control Hub asks the camera for its newest result (1-250 Hz).
        limelight.setPollRateHz(BioBuzzConstants.Vision.POLL_RATE_HZ);
        // Make sure the camera is running the AprilTag pipeline, not whatever was selected last time.
        limelight.pipelineSwitch(BioBuzzConstants.Vision.PIPELINE_INDEX);
        limelight.start();
    }

    /** Stops polling the camera (call when the OpMode ends). */
    public void stop() {
        try {
            limelight.stop();
        } catch (RuntimeException ignored) {
            // shutting down; nothing useful to do
        }
    }

    /** True if the Control Hub can currently talk to the Limelight (USB cable and power are fine). */
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

    /**
     * The real work of {@link #read}: run the result past a series of health checks (each failure becomes a
     * {@link VisionFrame.Status} that says what is wrong), then convert every tag in it.
     */
    private VisionFrame readInternal(double nowSec) {
        if (!limelight.isConnected()) {
            return VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, nowSec);
        }
        LLResult r = limelight.getLatestResult();
        if (r == null) {
            return VisionFrame.unavailable(VisionFrame.Status.NO_RESULT, nowSec);
        }

        // Is this a frame we have not seen before? The SDK hands back the same result object until the next
        // poll, so the timestamp is the only way to tell. (Only whether it changed matters, not its units.)
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
            // The camera still looks connected but has stopped producing frames: do not keep trusting the last one.
            return VisionFrame.unavailable(VisionFrame.Status.STALLED, nowSec);
        }

        // The AprilTag pipeline must be the one running, otherwise the results mean something else.
        if (r.getPipelineIndex() != BioBuzzConstants.Vision.PIPELINE_INDEX) {
            return VisionFrame.unavailable(VisionFrame.Status.WRONG_PIPELINE, nowSec);
        }

        // How old is the picture? Staleness (time since the hub last heard from the camera) plus the time the
        // camera took to capture and to process it, all in milliseconds. Too old = stale frame = ignore.
        double ageMs = r.getStaleness() + r.getCaptureLatency() + r.getTargetingLatency();
        if (ageMs > BioBuzzConstants.Vision.MAX_FRAME_AGE_MS) {
            return VisionFrame.unavailable(VisionFrame.Status.STALE, nowSec);
        }

        // Healthy frame: convert each tag. ("Fiducial" is the SDK's word for an AprilTag.)
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

    /**
     * Turns one Limelight tag into a {@link TagObservation} in our units and frames, or returns null if the
     * tag should be ignored (its measured distance is outside the believable range).
     */
    private TagObservation convert(LLResultTypes.FiducialResult f) {
        // tx / ty: how many degrees the tag is from the center of the image (the crosshair), sideways and up/down.
        // The switches let one constant fix a camera whose sign convention turns out to be the opposite; after
        // this, everything else can rely on "tx positive = right, ty positive = up".
        double tx = f.getTargetXDegrees();
        double ty = f.getTargetYDegrees();
        if (!BioBuzzConstants.Vision.TX_POSITIVE_IS_RIGHT) {
            tx = -tx;
        }
        if (!BioBuzzConstants.Vision.TY_POSITIVE_IS_UP) {
            ty = -ty;
        }
        // The direction to the tag on the floor plan works from tx/ty alone, so even a tag without a 3D position
        // still has a usable bearing.
        double azimuth = camera.azimuthOf(tx, ty);
        double area = f.getTargetArea();

        // The tag's 3D position in camera space (needs "Full 3D" ON in the pipeline). A missing or all-zero
        // position means there is no 3D data, so fall back to a bearing-only tag.
        Pose3D pose = f.getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null) {
            return TagObservation.bearingOnly(f.getFiducialId(), tx, ty, area, azimuth);
        }
        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        rawSummary.append(String.format("id%d raw=(%.1f, %.1f, %.1f) in; ", f.getFiducialId(), p.x, p.y, p.z));
        if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) {
            return TagObservation.bearingOnly(f.getFiducialId(), tx, ty, area, azimuth);   // "Full 3D" is off
        }

        // Rename the Limelight's axes into (right, down, forward). The Limelight docs disagree about which
        // convention camera-space poses use, so the choice is a constant that the diagnostic OpMode checks.
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
        // Straight-line distance from the lens. A tag outside the believable range (TAG_MIN_RANGE_IN to
        // TAG_MAX_RANGE_IN; far tags in particular are hard to read) is dropped rather than trusted.
        double distance = Math.sqrt(right * right + down * down + forward * forward);
        if (distance < BioBuzzConstants.Vision.TAG_MIN_RANGE_IN || distance > BioBuzzConstants.Vision.TAG_MAX_RANGE_IN) {
            rejectedRange++;
            return null;
        }
        // Move from "relative to the lens" to "relative to the robot's center" using the camera mount numbers.
        double[] robot = camera.cameraToRobot(right, down, forward);
        return TagObservation.withPosition(f.getFiducialId(), tx, ty, area, azimuth, robot[0], robot[1], robot[2]);
    }

    // ------------------------------------------------------------------------------ diagnostics

    /** How many different camera frames have been seen since the program started. */
    public int framesSeen() {
        return framesSeen;
    }

    /** How many tags have been dropped for being closer than TAG_MIN_RANGE_IN or farther than TAG_MAX_RANGE_IN. */
    public int rejectedForRange() {
        return rejectedRange;
    }

    /** The last error {@link #read} caught (empty if none), for the telemetry screen. */
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
