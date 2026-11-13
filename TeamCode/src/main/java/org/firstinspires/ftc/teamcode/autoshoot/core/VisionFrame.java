package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * One loop's snapshot of the camera. Built fresh every loop by the Limelight adapter; the logic only
 * ever sees this immutable object, so it can be tested with hand-made frames.
 *
 * <p>A "frame" here means "what the Limelight said this loop": a status (is the camera healthy and is the
 * answer fresh?) and the list of tags it saw. The robot loop runs much faster than the camera takes pictures,
 * so in most loops the camera has nothing new to say and {@link #newFrame} is false. A frame that is too old to
 * trust is called STALE; the status says which problem it is, so the telemetry can tell the driver.
 *
 * <p>Data flow: {@code LimelightVision.read} creates it, then {@link ClusterAnalyzer}, {@link HiveTracker}
 * and {@link PoseCorrector} read it.
 */
public final class VisionFrame {
    /** Why the frame carries no usable tags. {@code OK} means it may carry tags (possibly an empty list). */
    public enum Status {
        /** The camera is connected, the result is fresh and from the right pipeline. The tag list may be empty. */
        OK,
        /** The Control Hub cannot reach the Limelight (cable, power, or no Limelight in the TeleOp). */
        NOT_CONNECTED,
        /** The camera is connected but has produced no result yet (or reading it threw an error). */
        NO_RESULT,
        /** The newest result is older than {@code Vision.MAX_FRAME_AGE_MS}: the picture is too old to use. */
        STALE,
        /** The link looks alive but no NEW frame has arrived for {@code Vision.FRAME_STALL_MS}: vision is frozen. */
        STALLED,
        /** The camera is running a pipeline other than the AprilTag one ({@code Vision.PIPELINE_INDEX}). */
        WRONG_PIPELINE
    }

    /** Whether this snapshot can be used at all; anything other than OK means the tag list is empty. */
    public final Status status;
    /** True only on the loop in which a camera frame the logic has not seen before arrived. */
    public final boolean newFrame;
    /** Estimated age of the image when it was read, in seconds. */
    public final double ageSec;
    /** Time the logic's clock read when this snapshot was taken, in seconds. */
    public final double nowSec;
    /** The AprilTags seen, in the robot frame, unchangeable. Empty if none are in view or {@link #usable} is false. */
    public final List<TagObservation> tags;

    public VisionFrame(Status status, boolean newFrame, double ageSec, double nowSec, List<TagObservation> tags) {
        this.status = status;
        this.newFrame = newFrame;
        this.ageSec = ageSec;
        this.nowSec = nowSec;
        // Copy the list, then wrap it read-only, so nobody can change a frame after it was made.
        this.tags = Collections.unmodifiableList(new ArrayList<TagObservation>(tags));
    }

    /** A frame with a problem status and no tags. {@code newFrame} is false and {@code ageSec} is a meaningless 0. */
    public static VisionFrame unavailable(Status status, double nowSec) {
        return new VisionFrame(status, false, 0.0, nowSec, Collections.<TagObservation>emptyList());
    }

    /** True if tags in this frame may be trusted at all (link healthy and the data is fresh). */
    public boolean usable() {
        return status == Status.OK;
    }
}
