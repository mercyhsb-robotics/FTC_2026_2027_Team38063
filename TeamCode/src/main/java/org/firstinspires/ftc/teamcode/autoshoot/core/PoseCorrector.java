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
 *
 * <p>Words used here: "odometry" is the robot working out where it is by counting wheel/encoder movement; the
 * "Pinpoint" is the goBILDA odometry computer that does this for us; "drift" is the small error that builds up
 * the longer the robot drives; the "implied position" is the robot position that {@link ClusterAnalyzer}
 * computed from the tags (see {@link ClusterView#impliedX}). "Gate" means a limit beyond which a reading is
 * rejected instead of used.
 *
 * <p>Data flow: the TeleOp calls {@link #update} once per loop with the Pinpoint pose, the robot's speed, the
 * newest {@link VisionFrame} and the {@link ClusterView}s. If it gets a {@link Correction} back it writes the new
 * position into the odometry ({@code PedroDrive.setPose}). The TeleOp skips this class while a ball is being fed,
 * so the robot's position never jumps in the middle of a shot. Only after the camera mount constants have been
 * checked on the robot is it safe to turn on; see docs/LIMELIGHT_SETUP.md section 4, step 9.
 */
public final class PoseCorrector {

    /** The settings; the defaults come from {@link BioBuzzConstants}. */
    public static final class Config {
        /** Master switch. When false {@link #update} always returns null. */
        public boolean enabled;
        /** Fraction (0-1) of the disagreement corrected each time. */
        public double gain;
        /** Largest single correction, in inches. */
        public double maxStepIn;
        /** A sighting disagreeing with odometry by more than this (inches) is assumed wrong and ignored. */
        public double gateIn;
        /** Only correct while the robot moves slower than this (inches per second). */
        public double maxSpeedInS;
        /** ... and turns slower than this (degrees per second). */
        public double maxOmegaDegS;
        /** ... and the camera image is no older than this (seconds). */
        public double maxFrameAgeSec;
        /** At least this many of the cluster's four tags must be in view. */
        public int minTags;
        /** At least this long (seconds) between two corrections. */
        public double minIntervalSec;

        /** The values in {@code BioBuzzConstants.Vision.POSE_CORRECTION_*}. */
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
        /** Amount to ADD to the odometry x, inches (field frame: +x toward the blue wall). */
        public final double dx;
        /** Amount to ADD to the odometry y, inches (field frame: +y away from the audience). */
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
    // When we last applied a correction (the logic clock, seconds); -infinity so the first one is never blocked.
    private double lastAppliedSec = Double.NEGATIVE_INFINITY;
    // Counters and the last reason, kept only so the driver's screen can show what the corrector is doing.
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
     * @param nowSec    the logic clock, seconds
     * @param pose      where odometry says the robot is now (field frame: inches, radians)
     * @param speedInS  the robot's translation speed, inches per second
     * @param omegaRadS the robot's turn rate, radians per second (the sign does not matter)
     * @param frame     this loop's camera snapshot
     * @param views     {@link ClusterAnalyzer} output for that frame
     * @return the step to add to the odometry position, or null if no correction should be made now
     */
    public Correction update(double nowSec, Pose2 pose, double speedInS, double omegaRadS,
                             VisionFrame frame, List<ClusterView> views) {
        // Each check below is a reason to do nothing this loop; the reason is kept for the driver's screen.
        if (!cfg.enabled) {
            lastReason = "off (POSE_CORRECTION_ENABLED is false)";
            return null;
        }
        if (!frame.usable()) {
            lastReason = "camera is not delivering images";
            return null;
        }
        if (!frame.newFrame) {
            return null;                                   // the same image as last loop: nothing new to learn from
        }
        // The camera image is a little old by the time we read it; if the robot is still moving, that old
        // picture shows where the robot WAS, so we only trust it when the image is fresh and the robot is slow.
        if (frame.ageSec > cfg.maxFrameAgeSec) {
            lastReason = "image too old";
            return null;
        }
        if (speedInS > cfg.maxSpeedInS || Math.toDegrees(Math.abs(omegaRadS)) > cfg.maxOmegaDegS) {
            lastReason = "robot moving";
            return null;
        }
        // Writing a new position to the odometry costs an I2C transaction, so do not do it every loop.
        if (nowSec - lastAppliedSec < cfg.minIntervalSec) {
            return null;
        }
        ClusterView best = bestView(views);
        if (best == null) {
            lastReason = "no usable cluster in view";
            return null;
        }

        // How far (and in which direction, field x/y inches) the camera's answer is from odometry's.
        double ex = best.impliedX - pose.x;
        double ey = best.impliedY - pose.y;
        double disagreement = Math.hypot(ex, ey);
        lastDisagreementIn = disagreement;
        // A big disagreement is more likely a bad reading (or a wrong camera mount) than real drift: ignore it
        // rather than drag the robot across the field.
        if (disagreement > cfg.gateIn) {
            rejectedTooFar++;
            lastReason = "ignored: camera disagrees by " + Math.round(disagreement) + " in";
            return null;
        }
        // Move only a fraction of the way (gain), capped at maxStepIn; scale the (ex, ey) arrow to that length.
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

    /** How many corrections have been handed out so far (for the driver's screen). */
    public int appliedCount() {
        return applied;
    }

    /** How many sightings were thrown away for disagreeing by more than the gate. */
    public int rejectedCount() {
        return rejectedTooFar;
    }

    /** A short sentence for the driver's screen. */
    public String status() {
        return lastReason + "  (applied " + applied + ", last disagreement "
                + (Double.isNaN(lastDisagreementIn) ? "-" : String.format("%.1f in", lastDisagreementIn)) + ")";
    }
}
