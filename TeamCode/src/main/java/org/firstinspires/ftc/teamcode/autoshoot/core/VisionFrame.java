package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * One loop's snapshot of the camera. Built fresh every loop by the Limelight adapter; the logic only
 * ever sees this immutable object, so it can be tested with hand-made frames.
 */
public final class VisionFrame {
    /** Why the frame carries no usable tags. {@code OK} means it may carry tags (possibly an empty list). */
    public enum Status {
        OK,
        NOT_CONNECTED,
        NO_RESULT,
        STALE,
        STALLED,
        WRONG_PIPELINE
    }

    public final Status status;
    /** True only on the loop in which a camera frame the logic has not seen before arrived. */
    public final boolean newFrame;
    /** Estimated age of the image when it was read, in seconds. */
    public final double ageSec;
    /** Time the logic's clock read when this snapshot was taken, in seconds. */
    public final double nowSec;
    public final List<TagObservation> tags;

    public VisionFrame(Status status, boolean newFrame, double ageSec, double nowSec, List<TagObservation> tags) {
        this.status = status;
        this.newFrame = newFrame;
        this.ageSec = ageSec;
        this.nowSec = nowSec;
        this.tags = Collections.unmodifiableList(new ArrayList<TagObservation>(tags));
    }

    public static VisionFrame unavailable(Status status, double nowSec) {
        return new VisionFrame(status, false, 0.0, nowSec, Collections.<TagObservation>emptyList());
    }

    /** True if tags in this frame may be trusted at all (link healthy and the data is fresh). */
    public boolean usable() {
        return status == Status.OK;
    }
}
