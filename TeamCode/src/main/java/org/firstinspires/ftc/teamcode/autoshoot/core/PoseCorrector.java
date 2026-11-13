package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Uses the camera to pull the odometry position back toward the truth.
 *
 * <p>Odometry (the Pinpoint) is very good over a short distance but slowly drifts: wheels slip, the
 * heading creeps. Every time the Limelight sees the HIVE's AprilTags, {@link ClusterAnalyzer} works out where the
 * robot must be for the tags to look that way (using the robot's heading from odometry). If that disagrees a little
 * with where odometry says the robot is, the difference is mostly drift, so this class returns a small step toward the
 * camera's answer.
 *
 * <p>It is deliberately cautious:
 * <ul>
 *   <li>It only corrects the POSITION. The heading is left to the Pinpoint's gyro.</li>
 *   <li>A sighting that disagrees by more than a gate distance is thrown away as an obvious mistake.</li>
 *   <li>Each correction is only a fraction of the disagreement, limited in size, so one bad reading cannot move
 *       the robot much.</li>
 *   <li>It only works while the robot is nearly still (the picture is slightly old), with a fresh frame, at least a
 *       few of the four tags visible, and not more often than a few times a second.</li>
 *   <li>It is off unless {@code BioBuzzConstants.Vision.POSE_CORRECTION_ENABLED} is true.</li>
 * </ul>
 */
public final class PoseCorrector {

    /** The settings; the defaults come from {@link BioBuzzConstants}. */
    public static final class Config {
        public boolean enabled;
        public double gain;
        public double maxStepIn;
        public double gateIn;
        public double maxSpeedInS;
        public double maxOmegaDegS;
        public double maxFrameAgeSec;
        public int minTags;
        public double minIntervalSec;

        public static Config fromConstants() {
            Config c = new Config();
            c.enabled = BioBuzzConstants.Vision.POSE_CORRECTION_ENABLED;
            c.gain = BioBuzzConstants.Vision.POSE_CORRECTION_GAIN;
            c.maxStepIn = BioBuzzConstants.Vision.POSE_CORRECTION_MAX_STEP_IN;
            c.gateIn = BioBuzzConstants.Vision.POSE_CORRECTION_GATE_IN;
            c.maxSpeedInS = BioBuzzConstants.Vision.POSE_CORRECTION_MAX_SPEED_IN_S;
            c.maxOmegaDegS = BioBuzzConstants.Vision.POSE_CORRECTION_MAX_OMEGA_DEG_S;
            c.maxFrameAgeSec = BioBuzzConstants.Vision.POSE_CORRECTION_MAX_FRAME_AGE_SEC;
            c.minTags = BioBuzzConstants.Vision.POSE_CORRECTION_MIN_TAGS;
            c.minIntervalSec = BioBuzzConstants.Vision.POSE_CORRECTION_MIN_INTERVAL_SEC;
            return c;
        }
    }

    /** A step to add to the odometry position, in inches. */
    public static final class Correction {
        public final double dx;
        public final double dy;
        /** How far odometry and the camera disagreed before this step, in inches. */
        public final double disagreementIn;

        Correction(double dx, double dy, double disagreementIn) {
            this.dx = dx;
            this.dy = dy;
            this.disagreementIn = disagreementIn;
        }
    }

    private final Config cfg;
    private double lastAppliedSec = Double.NEGATIVE_INFINITY;
    private int applied;
    private int rejectedTooFar;
    private double lastDisagreementIn = Double.NaN;
    private String lastReason = "no sighting yet";

    public PoseCorrector(Config cfg) {
        this.cfg = cfg;
    }

    /**
     * Call once per loop.
     *
     * @return the step to add to the odometry position, or null if no correction should be made now
     */
    public Correction update(double nowSec, Pose2 pose, double speedInS, double omegaRadS,
                             VisionFrame frame, List<ClusterView> views) {
        if (!cfg.enabled) {
            lastReason = "off (POSE_CORRECTION_ENABLED is false)";
            return null;
        }
        if (!frame.usable() || !frame.newFrame) {
            return null;                                   // nothing new to learn from
        }
        if (frame.ageSec > cfg.maxFrameAgeSec) {
            lastReason = "image too old";
            return null;
        }
        if (speedInS > cfg.maxSpeedInS || Math.toDegrees(Math.abs(omegaRadS)) > cfg.maxOmegaDegS) {
            lastReason = "robot moving";
            return null;
        }
        if (nowSec - lastAppliedSec < cfg.minIntervalSec) {
            return null;
        }
        ClusterView best = bestView(views);
        if (best == null) {
            lastReason = "no usable cluster in view";
            return null;
        }

        double ex = best.impliedX - pose.x;
        double ey = best.impliedY - pose.y;
        double disagreement = Math.hypot(ex, ey);
        lastDisagreementIn = disagreement;
        if (disagreement > cfg.gateIn) {
            rejectedTooFar++;
            lastReason = "ignored: camera disagrees by " + Math.round(disagreement) + " in";
            return null;
        }
        double step = Math.min(cfg.maxStepIn, cfg.gain * disagreement);
        double scale = disagreement > 1e-9 ? step / disagreement : 0.0;
        lastAppliedSec = nowSec;
        applied++;
        lastReason = "applied";
        return new Correction(ex * scale, ey * scale, disagreement);
    }

    /** The cluster with the most tags among those that are UP and have a full position estimate, or null. */
    private ClusterView bestView(List<ClusterView> views) {
        ClusterView best = null;
        for (ClusterView v : views) {
            if (v.orientation == ClusterView.Orientation.UP && v.hasGeometry && v.tagCount >= cfg.minTags
                    && (best == null || v.tagCount > best.tagCount)) {
                best = v;
            }
        }
        return best;
    }

    public int appliedCount() {
        return applied;
    }

    public int rejectedCount() {
        return rejectedTooFar;
    }

    /** A short sentence for the driver's screen. */
    public String status() {
        return lastReason + "  (applied " + applied + ", last disagreement "
                + (Double.isNaN(lastDisagreementIn) ? "-" : String.format("%.1f in", lastDisagreementIn)) + ")";
    }
}
