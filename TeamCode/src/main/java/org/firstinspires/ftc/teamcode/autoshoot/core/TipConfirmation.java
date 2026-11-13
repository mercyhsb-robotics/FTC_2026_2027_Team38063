package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Decides, from the CAMERA and only the camera, that a HIVE has FULLY tipped and the cell the robot wants to shoot at is now UP.
 *
 * <p><b>Why it exists.</b> In the autonomous period the robot may have to shoot at the cell that comes up after the alliance
 * partner's tip (or after its own earlier tip). Partner teams tip at different times, so the robot must never shoot because a
 * clock says "the tip should be done by now": a POLLEN launched at a cell that still faces down is a foul (rule G417), and
 * launching at a HIVE that is still rocking can spoil the tip. The only safe evidence is what the camera sees.
 *
 * <p><b>"Fully tipped" means ALL of these at once</b> (the user's fourth constraint):
 * <ol>
 *   <li>the target cell's AprilTag cluster is in view,</li>
 *   <li>the analyzer reads it as UP with a full geometry (range and bearing, not just "some tags"),</li>
 *   <li>the {@link HiveTracker}'s belief is that this cell is UP and it got there from CAMERA evidence
 *       ({@link HiveTracker#cameraConfirmed()}): the driver's button and the "tags never seen, assume the other side" flip do
 *       not count (nobody drives in AUTO, and a guess is not a tip),</li>
 *   <li>and the cluster has been seen like that without a break for the settle time ({@code Vision.HIVE_STABLE_SEEN_SEC}), so a
 *       HIVE that is still rocking (its tags swing in and out of view) is not shot at. A TIP is complete only when the damper
 *       touches the frame.</li>
 * </ol>
 *
 * <p><b>No camera, no confirmation.</b> If the camera is missing, not delivering frames, stale or on the wrong pipeline, the state is
 * {@link State#NO_CAMERA}, the settle clock is thrown away and nothing is ever confirmed. The robot then must not shoot any volley
 * that depends on the tip: it skips it and carries on with the safe steps (LEAVE, PARK). A missing frame also breaks "seen without
 * a break", so a camera that blinks starts the settle time again (fail closed).
 *
 * <p><b>How to use it.</b> Every loop, in this order: call {@link HiveTracker#update} with the frame, then {@link #update} with the
 * same frame and the cluster views. The confirmation is LATCHED: once it says CONFIRMED it stays confirmed (the volley's own shot
 * gate, {@code AutoShootController}, keeps watching the cluster while it fires). Time comes in as a parameter, never a clock, so a
 * test can replay any sequence of frames.
 */
public final class TipConfirmation {

    /** The tuning number: how long the cluster must be seen without a break. Defaults from {@code BioBuzzConstants.Vision}. */
    public static final class Config {
        /** Seconds the target cluster must be seen UP, with full geometry, in every new frame, before the tip counts as complete. */
        public double settleSec;

        /** The values in {@code BioBuzzConstants.Vision}. */
        public static Config fromConstants() {
            Config c = new Config();
            c.settleSec = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
            return c;
        }
    }

    /** What the camera has proved so far. */
    public enum State {
        /** The camera works but has not yet shown the target cell UP, steady, for the settle time. */
        WAITING,
        /** The HIVE has fully tipped: the target cell is UP and has been seen steady for the settle time. */
        CONFIRMED,
        /** The camera cannot be used right now (missing, stalled, stale or on the wrong pipeline): nothing can be confirmed. */
        NO_CAMERA
    }

    private final HiveTracker tracker;
    private final Side target;
    private final Config cfg;

    private State state = State.WAITING;
    /** Route time since which every new frame has shown the target cluster UP with full geometry; NaN = no such run now. */
    private double seenSince = Double.NaN;
    private double confirmedAtSec = Double.NaN;
    private boolean everSawCamera;
    private boolean everSawCluster;

    /**
     * @param tracker the tracker that is fed the same frames (it must be updated before this class, every loop)
     * @param target  the cell of our alliance's HIVE the robot wants to shoot at once it is UP
     */
    public TipConfirmation(HiveTracker tracker, Side target, Config cfg) {
        this.tracker = tracker;
        this.target = target;
        this.cfg = cfg;
    }

    /** The cell this confirmation is waiting to see UP. */
    public Side target() {
        return target;
    }

    /** The current verdict. */
    public State state() {
        return state;
    }

    /** True once the HIVE has fully tipped (latched). */
    public boolean confirmed() {
        return state == State.CONFIRMED;
    }

    /** The time (the clock the caller passes in) at which the tip was confirmed; NaN while it is not. */
    public double confirmedAtSec() {
        return confirmedAtSec;
    }

    /**
     * The time before which the robot must stay out of the spill zone of the cell that tipped (rule G409): the moment the camera
     * confirmed the tip plus {@code Auto.TIP_SPILL_CLEAR_SEC}. NaN while the tip is not confirmed (then the robot must stay out).
     */
    public double spillClearAtSec() {
        return Double.isNaN(confirmedAtSec) ? Double.NaN : confirmedAtSec + BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC;
    }

    /** How long the cluster has been seen UP without a break as of {@code nowSec}; 0 if it is not being seen like that now. */
    public double seenSteadySec(double nowSec) {
        return Double.isNaN(seenSince) ? 0.0 : Math.max(0.0, nowSec - seenSince);
    }

    /**
     * Call once per loop, after {@link HiveTracker#update} with the same frame.
     *
     * @param nowSec the logic clock
     * @param frame  this loop's camera snapshot
     * @param views  {@link ClusterAnalyzer} output for this frame (ignored unless the frame is usable)
     */
    public State update(double nowSec, VisionFrame frame, List<ClusterView> views) {
        if (state == State.CONFIRMED) {
            return state;
        }
        if (!frame.usable()) {
            // Without a working camera nothing can be confirmed, and "seen without a break" is lost.
            seenSince = Double.NaN;
            state = State.NO_CAMERA;
            return state;
        }
        everSawCamera = true;
        if (state == State.NO_CAMERA) {
            state = State.WAITING;
        }
        // Only a NEW frame says anything about whether the cluster is still there; repeats of the old one add nothing.
        if (frame.newFrame) {
            if (targetClusterIsUp(views)) {
                everSawCluster = true;
                if (Double.isNaN(seenSince)) {
                    seenSince = nowSec;
                }
            } else {
                seenSince = Double.NaN;          // a frame without it breaks the run: a rocking HIVE is not a fully tipped one
            }
        }
        boolean beliefFromCamera = tracker.upSide() == target && tracker.confirmed() && tracker.cameraConfirmed();
        if (beliefFromCamera && !Double.isNaN(seenSince) && nowSec - seenSince >= cfg.settleSec) {
            state = State.CONFIRMED;
            confirmedAtSec = nowSec;
        }
        return state;
    }

    /** True if one of the views is the target cell's cluster, reads UP and has its full geometry (range and bearing). */
    private boolean targetClusterIsUp(List<ClusterView> views) {
        for (ClusterView v : views) {
            if (v.cell.side == target && v.orientation == ClusterView.Orientation.UP && v.hasGeometry) {
                return true;
            }
        }
        return false;
    }

    /**
     * Why the tip is not confirmed yet, in plain words for the driver screen and the log (empty once it is confirmed): no camera, the
     * cluster never seen, the belief not from the camera, or how much of the settle time has passed.
     */
    public String why(double nowSec) {
        if (state == State.CONFIRMED) {
            return "";
        }
        if (state == State.NO_CAMERA) {
            return everSawCamera ? "the camera stopped delivering frames, so nothing can be confirmed" : "no camera, so nothing can be confirmed";
        }
        if (!everSawCluster) {
            return "the camera has not yet seen the " + target + " cell's tags UP";
        }
        if (Double.isNaN(seenSince)) {
            return "the " + target + " cell's tags are not in view steadily (the HIVE may still be rocking)";
        }
        if (!(tracker.upSide() == target && tracker.confirmed() && tracker.cameraConfirmed())) {
            return "the tracker does not yet believe, from the camera, that the " + target + " cell is UP";
        }
        return String.format(Locale.ROOT, "the %s cell's tags have been steady for %.2f of %.2f s", target, seenSteadySec(nowSec), cfg.settleSec);
    }
}
