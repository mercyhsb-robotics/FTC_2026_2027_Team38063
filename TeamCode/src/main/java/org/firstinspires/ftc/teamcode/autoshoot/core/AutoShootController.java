package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The hold-a-button "drive to the scoring pose and shoot everything" sequence, as a pure state machine.
 *
 * <p>Each loop the caller passes in everything that was measured ({@link Inputs}) and gets back
 * everything to command ({@link Outputs}). Nothing here touches hardware, a clock or the SDK, so the
 * whole sequence can be simulated and tested on a PC.
 *
 * <pre>
 *   IDLE --press--> PLAN --> APPROACH --> ALIGN --> FEED --> DONE
 *                     ^          |           |        |
 *                     +----------+-----------+--------+   the HIVE tipped / the other cell is up
 *   any state --release--> IDLE        any state --problem--> FAULT(reason), wait for release
 * </pre>
 *
 * <ul>
 *   <li><b>PLAN</b> picks the cell the {@link HiveTracker} says is UP and asks the {@link ScoringPlanner}
 *       for the closest feasible pose. The flywheel starts spinning for that range immediately.</li>
 *   <li><b>APPROACH</b> drives there (around the HIVE frame if needed) and hands over on MEASURED
 *       pose error, never on a path-finished flag.</li>
 *   <li><b>ALIGN</b> holds the pose; the camera trims the heading toward the aim point and the range
 *       toward the planned range; it waits for the robot to settle.</li>
 *   <li><b>FEED</b> runs the feeder only while a fail-closed gate ({@link Refusal}) says every
 *       condition is met, counts shots, and stops the moment the up cell's tags vanish (a tip).</li>
 * </ul>
 * Releasing the button returns to manual driving, feeder off, in the same loop.
 */
public final class AutoShootController {

    public enum State { IDLE, PLAN, APPROACH, ALIGN, FEED, DONE, FAULT }

    /** Why the sequence gave up. */
    public enum Fault {
        NONE, NO_POSE, NO_FEASIBLE_POSE, APPROACH_TIMEOUT, APPROACH_STALLED, ALIGN_TIMEOUT, SPINUP_TIMEOUT,
        DRIVER_OVERRIDE, HIVE_NOT_VISIBLE, HIVE_UNCONFIRMED, FEED_TIMEOUT
    }

    /** The first reason the shot gate is closed, in priority order (permanent causes before transient ones). */
    public enum Refusal {
        NONE, POSE_UNTRUSTED, HIVE_UNCONFIRMED, TAGS_LOST, OUT_OF_RANGE, ROBOT_MOVING, HEADING_ERROR,
        FLYWHEEL_NOT_AT_SPEED, VOLLEY_COMPLETE
    }

    /** One-shot notifications for the driver (rumble), fired on state entry. */
    public enum Cue { NONE, STARTED, ARRIVED, DONE, FAULT }

    public enum DriveMode { MANUAL, FOLLOW, HOLD }

    // ------------------------------------------------------------------------------------------

    /** Every number the sequence uses; defaults come from {@link BioBuzzConstants}. */
    public static final class Config {
        public double handoffIn, handoffDeg, handoffSpeed, approachTimeoutSpeed, approachTimeoutExtra;
        public double stallSpeed, stallSec, stallGraceSec, stallMinDist;
        public double alignPosTol, alignHeadingTolDeg, alignSpeedTol, alignOmegaTolDegS, alignSettleSec, alignTimeoutSec;
        public double reapproachIn;
        public boolean visionTrim;
        public double headingGain, headingStepDeg, headingTrimMaxDeg, rangeGain, rangeStepIn, rangeTrimMaxIn, trimPeriodSec;
        public double shotMaxSpeed, shotMaxOmegaDegS, shotHeadingTolDeg, shotRelaxedPosTol, spinupTimeoutSec;
        public double visionLossPauseSec, hiveUnseenFlipSec;
        public boolean requireHiveConfirmation;
        public int maxUnseenFlips;
        public double overrideStick, overrideSec;
        public double launchForwardIn, launchLeftIn, rangeMarginIn, idleTps;
        public int maxShots;
        public BioBuzzConstants.Shooter.ShotCountMode countMode;
        public BioBuzzConstants.Shooter.FeedMode feedMode;
        public double feedSecondsPerShot, volleyMaxSec, emptyAfterLastShotSec;
        public double atBelow, atAbove, atFloor, atRise, atFall, dipFrac, refractory;
        public double maxLoopDtSec, trimEnterPosFactor, trimEnterHeadingDeg;

        public static Config fromConstants() {
            Config c = new Config();
            c.handoffIn = BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_IN;
            c.handoffDeg = BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_DEG;
            c.handoffSpeed = BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_SPEED_IN_S;
            c.approachTimeoutSpeed = BioBuzzConstants.AutoShoot.APPROACH_TIMEOUT_SPEED_IN_S;
            c.approachTimeoutExtra = BioBuzzConstants.AutoShoot.APPROACH_TIMEOUT_EXTRA_SEC;
            c.stallSpeed = BioBuzzConstants.AutoShoot.APPROACH_STALL_SPEED_IN_S;
            c.stallSec = BioBuzzConstants.AutoShoot.APPROACH_STALL_SEC;
            c.stallGraceSec = BioBuzzConstants.AutoShoot.APPROACH_STALL_GRACE_SEC;
            c.stallMinDist = BioBuzzConstants.AutoShoot.APPROACH_STALL_MIN_DIST_IN;
            c.alignPosTol = BioBuzzConstants.AutoShoot.ALIGN_POS_TOL_IN;
            c.alignHeadingTolDeg = BioBuzzConstants.AutoShoot.ALIGN_HEADING_TOL_DEG;
            c.alignSpeedTol = BioBuzzConstants.AutoShoot.ALIGN_SPEED_TOL_IN_S;
            c.alignOmegaTolDegS = BioBuzzConstants.AutoShoot.ALIGN_OMEGA_TOL_DEG_S;
            c.alignSettleSec = BioBuzzConstants.AutoShoot.ALIGN_SETTLE_SEC;
            c.alignTimeoutSec = BioBuzzConstants.AutoShoot.ALIGN_TIMEOUT_SEC;
            c.reapproachIn = BioBuzzConstants.AutoShoot.REAPPROACH_IN;
            c.visionTrim = BioBuzzConstants.AutoShoot.VISION_TRIM_ENABLED;
            c.headingGain = BioBuzzConstants.AutoShoot.VISION_HEADING_GAIN;
            c.headingStepDeg = BioBuzzConstants.AutoShoot.VISION_HEADING_STEP_DEG;
            c.headingTrimMaxDeg = BioBuzzConstants.AutoShoot.VISION_HEADING_TRIM_MAX_DEG;
            c.rangeGain = BioBuzzConstants.AutoShoot.VISION_RANGE_GAIN;
            c.rangeStepIn = BioBuzzConstants.AutoShoot.VISION_RANGE_STEP_IN;
            c.rangeTrimMaxIn = BioBuzzConstants.AutoShoot.VISION_RANGE_TRIM_MAX_IN;
            c.trimPeriodSec = BioBuzzConstants.AutoShoot.VISION_TRIM_PERIOD_SEC;
            c.shotMaxSpeed = BioBuzzConstants.AutoShoot.SHOT_MAX_SPEED_IN_S;
            c.shotMaxOmegaDegS = BioBuzzConstants.AutoShoot.SHOT_MAX_OMEGA_DEG_S;
            c.shotHeadingTolDeg = BioBuzzConstants.AutoShoot.SHOT_HEADING_TOL_DEG;
            c.shotRelaxedPosTol = BioBuzzConstants.AutoShoot.SHOT_RELAXED_POS_TOL_IN;
            c.spinupTimeoutSec = BioBuzzConstants.AutoShoot.SPINUP_TIMEOUT_SEC;
            c.visionLossPauseSec = BioBuzzConstants.AutoShoot.VISION_LOSS_PAUSE_SEC;
            c.hiveUnseenFlipSec = BioBuzzConstants.AutoShoot.HIVE_UNSEEN_FLIP_SEC;
            c.requireHiveConfirmation = BioBuzzConstants.AutoShoot.REQUIRE_HIVE_CONFIRMATION;
            c.maxUnseenFlips = BioBuzzConstants.AutoShoot.MAX_UNSEEN_FLIPS;
            c.overrideStick = BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_STICK;
            c.overrideSec = BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_SEC;
            c.launchForwardIn = BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN;
            c.launchLeftIn = BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN;
            c.rangeMarginIn = BioBuzzConstants.Shooter.RANGE_MARGIN_IN;
            c.idleTps = BioBuzzConstants.Shooter.FLYWHEEL_IDLE_TPS;
            c.maxShots = BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY;
            c.countMode = BioBuzzConstants.Shooter.SHOT_COUNT_MODE;
            c.feedMode = BioBuzzConstants.Shooter.FEED_MODE;
            c.feedSecondsPerShot = BioBuzzConstants.Shooter.FEED_SECONDS_PER_SHOT;
            c.volleyMaxSec = BioBuzzConstants.Shooter.VOLLEY_MAX_SEC;
            c.emptyAfterLastShotSec = BioBuzzConstants.Shooter.EMPTY_AFTER_LAST_SHOT_SEC;
            c.atBelow = BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC;
            c.atAbove = BioBuzzConstants.Shooter.AT_SPEED_ABOVE_FRAC;
            c.atFloor = BioBuzzConstants.Shooter.AT_SPEED_FLOOR_TPS;
            c.atRise = BioBuzzConstants.Shooter.AT_SPEED_RISE_SEC;
            c.atFall = BioBuzzConstants.Shooter.AT_SPEED_FALL_SEC;
            c.dipFrac = BioBuzzConstants.Shooter.SHOT_DIP_FRAC;
            c.refractory = BioBuzzConstants.Shooter.SHOT_REFRACTORY_SEC;
            c.maxLoopDtSec = BioBuzzConstants.AutoShoot.MAX_LOOP_DT_SEC;
            c.trimEnterPosFactor = BioBuzzConstants.AutoShoot.VISION_TRIM_ENTER_POS_FACTOR;
            c.trimEnterHeadingDeg = BioBuzzConstants.AutoShoot.VISION_TRIM_ENTER_HEADING_DEG;
            return c;
        }
    }

    /** Everything measured this loop. Build a fresh one every loop. */
    public static final class Inputs {
        public double nowSec;
        public boolean shootHeld;
        public Pose2 pose;
        /** Translation speed in inches per second. */
        public double speedInS;
        /** Turn rate in radians per second (absolute value is used). */
        public double omegaRadS;
        /** True once the odometry has been given a trustworthy starting pose. */
        public boolean poseTrusted;
        public double flywheelTps;
        public VisionFrame frame = VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, 0.0);
        /** {@link ClusterAnalyzer} output for this frame (own alliance only). */
        public List<ClusterView> views = Collections.emptyList();
        /** Magnitude (0-1) of the driver's translation stick. */
        public double stickMagnitude;
    }

    /** Everything to command. Defaults are the safe ones: manual drive, flywheel idle, feeder off. */
    public static final class Outputs {
        public DriveMode driveMode = DriveMode.MANUAL;
        /** For FOLLOW: the points to drive through; the last one is {@link #holdPose}'s position. */
        public List<double[]> waypoints = Collections.emptyList();
        /** For FOLLOW (the destination) and HOLD (the pose to hold). */
        public Pose2 holdPose;
        public double flywheelTargetTps;
        public boolean feederOn;
        public Cue cue = Cue.NONE;
        // Information for telemetry.
        public State state = State.IDLE;
        public Fault fault = Fault.NONE;
        public Refusal refusal = Refusal.NONE;
        public int shotsFired;
        public Side targetSide;
        public double rangeIn = Double.NaN;
        public double aimErrorDeg = Double.NaN;
        public String message = "";
    }

    // ------------------------------------------------------------------------------------------

    private final Alliance alliance;
    private final FieldModel field;
    private final ScoringPlanner planner;
    private final ShotTable table;
    private final HiveTracker tracker;
    private final Config cfg;
    private final FlywheelReady ready;
    private final ShotCounter counter;

    private State state = State.IDLE;
    private Fault fault = Fault.NONE;
    private boolean prevHeld;
    /** False until the button has been seen released once, so holding it through INIT cannot auto-start a volley. */
    private boolean releasedSeen = false;
    private double lastNow = Double.NaN;
    private double dt;
    private double stateSince;

    private Side targetSide;
    private CellSpec cell;
    private ScoringPlanner.Solution solution;
    private double headingTrimRad;
    private double rangeTrimIn;
    private double lastTrimSec = Double.NEGATIVE_INFINITY;
    private double approachTimeout;
    private double stallSince = Double.NaN;
    private double settledSince = Double.NaN;

    private int shotsBase;
    private double feedStart;
    private double feederOnSec;
    private boolean feedLatched;
    private double frozenTps;
    private double frozenRange;

    private int unseenFlips;
    private double unseenSince = Double.NaN;
    private double lastUpClusterSeen = Double.NEGATIVE_INFINITY;
    private boolean everSawCluster;
    private double overrideSince = Double.NaN;

    private int pressShots;

    public AutoShootController(Alliance alliance, FieldModel field, ScoringPlanner planner, ShotTable table,
                               HiveTracker tracker, Config cfg) {
        this.alliance = alliance;
        this.field = field;
        this.planner = planner;
        this.table = table;
        this.tracker = tracker;
        this.cfg = cfg;
        this.ready = new FlywheelReady(cfg.atBelow, cfg.atAbove, cfg.atFloor, cfg.atRise, cfg.atFall);
        this.counter = new ShotCounter(cfg.dipFrac, cfg.atFloor, cfg.refractory);
    }

    public State state() {
        return state;
    }

    public Fault fault() {
        return fault;
    }

    /** Shots fired since the button was pressed. */
    public int shotsFired() {
        return pressShots;
    }

    public ScoringPlanner.Solution solution() {
        return solution;
    }

    // ------------------------------------------------------------------------------------------

    public Outputs update(Inputs in) {
        dt = Double.isNaN(lastNow) ? 0.0 : Math.max(0.0, Math.min(cfg.maxLoopDtSec, in.nowSec - lastNow));
        lastNow = in.nowSec;

        boolean rising = in.shootHeld && !prevHeld;
        prevHeld = in.shootHeld;

        Outputs out = new Outputs();
        out.flywheelTargetTps = cfg.idleTps;

        if (!in.shootHeld) {
            releasedSeen = true;
            if (state != State.IDLE) {
                enter(State.IDLE, in.nowSec);
                fault = Fault.NONE;
                resetPress();
            }
            return finish(out, in);
        }

        if (state == State.IDLE) {
            if (rising && releasedSeen) {
                releasedSeen = false;
                begin(in);
            } else {
                return finish(out, in);
            }
        }

        // Safety checks that apply to every active state.
        if (state != State.DONE && state != State.FAULT) {
            if (!in.poseTrusted || in.pose == null) {
                fail(Fault.NO_POSE, in, out);
                return finish(out, in);
            }
            if (in.stickMagnitude > cfg.overrideStick) {
                if (Double.isNaN(overrideSince)) {
                    overrideSince = in.nowSec;
                }
                if (in.nowSec - overrideSince >= cfg.overrideSec) {
                    fail(Fault.DRIVER_OVERRIDE, in, out);
                    return finish(out, in);
                }
            } else {
                overrideSince = Double.NaN;
            }
            trackCluster(in);
        }

        switch (state) {
            case PLAN:
                plan(in, out);
                break;
            case APPROACH:
                approach(in, out);
                break;
            case ALIGN:
                align(in, out);
                break;
            case FEED:
                feed(in, out);
                break;
            case DONE:
                out.driveMode = DriveMode.HOLD;
                out.holdPose = in.pose;
                break;
            case FAULT:
            default:
                break;
        }
        return finish(out, in);
    }

    // ------------------------------------------------------------------------------------------ states

    private void begin(Inputs in) {
        resetPress();
        enter(State.PLAN, in.nowSec);
    }

    private void resetPress() {
        pressShots = 0;
        shotsBase = 0;
        unseenFlips = 0;
        unseenSince = Double.NaN;
        everSawCluster = false;
        lastUpClusterSeen = Double.NEGATIVE_INFINITY;
        overrideSince = Double.NaN;
        headingTrimRad = 0.0;
        rangeTrimIn = 0.0;
        solution = null;
        cell = null;
        targetSide = null;
        feedLatched = false;
        ready.reset();
    }

    private void enter(State next, double now) {
        state = next;
        stateSince = now;
        stallSince = Double.NaN;
        settledSince = Double.NaN;
        pendingCue = cueFor(next);
    }

    private Cue pendingCue = Cue.NONE;

    private static Cue cueFor(State s) {
        switch (s) {
            case PLAN:
                return Cue.STARTED;
            case ALIGN:
                return Cue.ARRIVED;
            case DONE:
                return Cue.DONE;
            case FAULT:
                return Cue.FAULT;
            default:
                return Cue.NONE;
        }
    }

    private void plan(Inputs in, Outputs out) {
        targetSide = tracker.upSide();
        cell = field.cell(alliance, targetSide);
        solution = planner.solve(in.pose, cell);
        if (!solution.ok()) {
            fail(Fault.NO_FEASIBLE_POSE, in, out);
            return;
        }
        headingTrimRad = 0.0;
        rangeTrimIn = 0.0;
        double dist = PathPlanner.length(in.pose.x, in.pose.y, solution.waypoints);
        approachTimeout = dist / cfg.approachTimeoutSpeed + cfg.approachTimeoutExtra;
        if (arrivedNear(in)) {
            enter(State.ALIGN, in.nowSec);
            align(in, out);
        } else {
            enter(State.APPROACH, in.nowSec);
            approach(in, out);
        }
    }

    private boolean arrivedNear(Inputs in) {
        double dist = in.pose.distanceTo(solution.pose);
        double headingErr = Math.toDegrees(Angles.distance(in.pose.heading, solution.pose.heading));
        return dist <= cfg.handoffIn && headingErr <= cfg.handoffDeg && in.speedInS <= cfg.handoffSpeed;
    }

    private void approach(Inputs in, Outputs out) {
        if (retargetIfHiveChanged(in, out)) {
            return;
        }
        out.driveMode = DriveMode.FOLLOW;
        out.waypoints = solution.waypoints;
        out.holdPose = solution.pose;
        out.flywheelTargetTps = table.velocityAt(odometryRange(in));

        double elapsed = in.nowSec - stateSince;
        double dist = in.pose.distanceTo(solution.pose);
        if (arrivedNear(in)) {
            enter(State.ALIGN, in.nowSec);
            align(in, out);
            return;
        }
        if (elapsed > approachTimeout) {
            fail(Fault.APPROACH_TIMEOUT, in, out);
            return;
        }
        if (elapsed > cfg.stallGraceSec && in.speedInS < cfg.stallSpeed && dist > cfg.stallMinDist) {
            if (Double.isNaN(stallSince)) {
                stallSince = in.nowSec;
            }
            if (in.nowSec - stallSince >= cfg.stallSec) {
                fail(Fault.APPROACH_STALLED, in, out);
            }
        } else {
            stallSince = Double.NaN;
        }
    }

    private Pose2 holdPose() {
        Pose2 base = solution.pose;
        double h = Angles.normalize(base.heading + headingTrimRad);
        double x = base.x - rangeTrimIn * Math.cos(base.heading);     // toward the target = along the shooter = backward
        double y = base.y - rangeTrimIn * Math.sin(base.heading);
        return new Pose2(x, y, h);
    }

    private void align(Inputs in, Outputs out) {
        if (retargetIfHiveChanged(in, out)) {
            return;
        }
        Pose2 hold = holdPose();
        out.driveMode = DriveMode.HOLD;
        out.holdPose = hold;
        out.flywheelTargetTps = table.velocityAt(bestRange(in));

        double posErr = in.pose.distanceTo(hold);
        double headErr = Math.toDegrees(Angles.distance(in.pose.heading, hold.heading));
        double omegaDegS = Math.toDegrees(Math.abs(in.omegaRadS));

        if (posErr > cfg.reapproachIn) {
            enter(State.APPROACH, in.nowSec);
            approachTimeout = posErr / cfg.approachTimeoutSpeed + cfg.approachTimeoutExtra;
            approach(in, out);
            return;
        }

        boolean nearHold = posErr <= cfg.trimEnterPosFactor * cfg.alignPosTol && headErr <= cfg.trimEnterHeadingDeg;
        if (cfg.visionTrim && nearHold) {
            applyVisionTrim(in);
            hold = holdPose();
            out.holdPose = hold;
            posErr = in.pose.distanceTo(hold);
            headErr = Math.toDegrees(Angles.distance(in.pose.heading, hold.heading));
        }

        boolean settled = posErr <= cfg.alignPosTol && headErr <= cfg.alignHeadingTolDeg
                && in.speedInS <= cfg.alignSpeedTol && omegaDegS <= cfg.alignOmegaTolDegS;
        if (settled) {
            if (Double.isNaN(settledSince)) {
                settledSince = in.nowSec;
            }
        } else {
            settledSince = Double.NaN;
        }

        boolean settledLongEnough = !Double.isNaN(settledSince) && in.nowSec - settledSince >= cfg.alignSettleSec;
        boolean timedOut = in.nowSec - stateSince >= cfg.alignTimeoutSec;
        if (settledLongEnough) {
            startFeed(in);
        } else if (timedOut) {
            double aimErrDeg = Math.toDegrees(Math.abs(ShotGeometry.aimError(in.pose, cfg.launchForwardIn,
                    cfg.launchLeftIn, cell.originX, cell.originY)));
            if (posErr <= cfg.shotRelaxedPosTol && aimErrDeg <= cfg.shotHeadingTolDeg) {
                startFeed(in);
            } else {
                fail(Fault.ALIGN_TIMEOUT, in, out);
            }
        }
    }

    /** Lets the camera nudge the hold heading toward the aim point and the hold range toward the planned range. */
    private void applyVisionTrim(Inputs in) {
        if (!in.frame.usable() || !in.frame.newFrame || in.nowSec - lastTrimSec < cfg.trimPeriodSec) {
            return;
        }
        ClusterView best = upViewFor(in);
        if (best == null) {
            return;
        }
        lastTrimSec = in.nowSec;
        if (best.hasBearing) {
            double step = Math.toRadians(cfg.headingStepDeg);
            double delta = Math.max(-step, Math.min(step, cfg.headingGain * best.aimErrorRad));
            headingTrimRad += delta;
            double max = Math.toRadians(cfg.headingTrimMaxDeg);
            headingTrimRad = Math.max(-max, Math.min(max, headingTrimRad));
        }
        if (best.hasGeometry) {
            double error = best.rangeIn - solution.rangeIn;
            double delta = Math.max(-cfg.rangeStepIn, Math.min(cfg.rangeStepIn, cfg.rangeGain * error));
            double next = Math.max(-cfg.rangeTrimMaxIn, Math.min(cfg.rangeTrimMaxIn, rangeTrimIn + delta));
            double saved = rangeTrimIn;
            rangeTrimIn = next;
            if (!planner.standingPoseFree(holdPose())) {
                rangeTrimIn = saved;                  // never trim into a wall or the HIVE frame
            }
        }
    }

    private ClusterView upViewFor(Inputs in) {
        for (ClusterView v : in.views) {
            if (v.cell == cell && v.orientation == ClusterView.Orientation.UP) {
                return v;
            }
        }
        return null;
    }

    private void startFeed(Inputs in) {
        enter(State.FEED, in.nowSec);
        feedStart = in.nowSec;
        feederOnSec = 0.0;
        feedLatched = false;
        ClusterView v = upViewFor(in);
        frozenRange = (v != null && v.hasGeometry && in.frame.usable()) ? v.rangeIn : odometryRange(in);
        frozenTps = table.velocityAt(frozenRange);
        counter.start(in.nowSec);
        shotsBase = pressShots;
    }

    private void feed(Inputs in, Outputs out) {
        if (retargetIfHiveChanged(in, out)) {
            return;
        }
        Pose2 hold = holdPose();
        out.driveMode = DriveMode.HOLD;
        out.holdPose = hold;
        out.flywheelTargetTps = frozenTps;
        out.rangeIn = frozenRange;

        // Pushed out of place: stop feeding and align again.
        if (in.pose.distanceTo(hold) > cfg.reapproachIn) {
            enter(State.ALIGN, in.nowSec);
            align(in, out);
            return;
        }

        boolean atSpeed = ready.update(in.nowSec, true, frozenTps, in.flywheelTps);
        Refusal refusal = evaluateGate(in, atSpeed);
        out.refusal = refusal;

        boolean open = refusal == Refusal.NONE;
        if (cfg.feedMode == BioBuzzConstants.Shooter.FeedMode.CONTINUOUS) {
            if (open) {
                feedLatched = true;
            }
            boolean hardStop = refusal != Refusal.NONE && refusal != Refusal.FLYWHEEL_NOT_AT_SPEED;
            out.feederOn = feedLatched && !hardStop && refusal != Refusal.VOLLEY_COMPLETE;
        } else {
            out.feederOn = open;
        }

        if (out.feederOn) {
            feederOnSec += dt;
        }
        counter.update(in.nowSec, out.feederOn, atSpeed, frozenTps, in.flywheelTps);

        int remaining = cfg.maxShots - shotsBase;
        int dips = counter.count();
        int timed = (int) Math.floor(feederOnSec / cfg.feedSecondsPerShot + 1e-9);
        int segmentShots;
        switch (cfg.countMode) {
            case TIMED:
                segmentShots = timed;
                break;
            case DIP:
            case HYBRID:
            default:
                segmentShots = dips;
                break;
        }
        segmentShots = Math.min(segmentShots, remaining);
        pressShots = shotsBase + segmentShots;

        double sinceFeedStart = in.nowSec - feedStart;
        if (pressShots >= cfg.maxShots) {
            enter(State.DONE, in.nowSec);
            out.feederOn = false;
            out.driveMode = DriveMode.HOLD;
            out.holdPose = in.pose;
            return;
        }
        if (cfg.countMode == BioBuzzConstants.Shooter.ShotCountMode.TIMED && timed >= remaining) {
            enter(State.DONE, in.nowSec);
            out.feederOn = false;
            return;
        }
        if (cfg.countMode == BioBuzzConstants.Shooter.ShotCountMode.HYBRID) {
            boolean hopperEmpty = dips >= 1 && counter.secondsSinceLastShot(in.nowSec) >= cfg.emptyAfterLastShotSec
                    && feederOnSec > 0.0 && out.feederOn;
            boolean dipsMissed = timed >= remaining + 1;
            if (hopperEmpty || dipsMissed) {
                enter(State.DONE, in.nowSec);
                out.feederOn = false;
                return;
            }
        }
        // The flywheel never got up to speed.
        if (feederOnSec == 0.0 && sinceFeedStart > cfg.spinupTimeoutSec) {
            fail(Fault.SPINUP_TIMEOUT, in, out);
            return;
        }
        if (sinceFeedStart > cfg.volleyMaxSec + cfg.spinupTimeoutSec) {
            if (cfg.countMode == BioBuzzConstants.Shooter.ShotCountMode.DIP && dips == 0) {
                fail(Fault.FEED_TIMEOUT, in, out);
            } else {
                enter(State.DONE, in.nowSec);
                out.feederOn = false;
            }
        }
    }

    /** The fail-closed gate: the first reason a ball may NOT be fed right now, or NONE. */
    private Refusal evaluateGate(Inputs in, boolean atSpeed) {
        if (!in.poseTrusted) {
            return Refusal.POSE_UNTRUSTED;
        }
        boolean cameraWorking = in.frame.usable();
        if (cfg.requireHiveConfirmation && cameraWorking && !tracker.confirmed()) {
            return Refusal.HIVE_UNCONFIRMED;
        }
        if (cameraWorking && everSawCluster && in.nowSec - lastUpClusterSeen >= cfg.visionLossPauseSec) {
            return Refusal.TAGS_LOST;
        }
        if (!table.inRange(frozenRange)) {
            return Refusal.OUT_OF_RANGE;
        }
        if (in.speedInS > cfg.shotMaxSpeed || Math.toDegrees(Math.abs(in.omegaRadS)) > cfg.shotMaxOmegaDegS) {
            return Refusal.ROBOT_MOVING;
        }
        double aimErr = Math.toDegrees(Math.abs(ShotGeometry.aimError(in.pose, cfg.launchForwardIn,
                cfg.launchLeftIn, cell.originX, cell.originY)));
        if (aimErr > cfg.shotHeadingTolDeg) {
            return Refusal.HEADING_ERROR;
        }
        if (!atSpeed) {
            return Refusal.FLYWHEEL_NOT_AT_SPEED;
        }
        if (pressShots >= cfg.maxShots) {
            return Refusal.VOLLEY_COMPLETE;
        }
        return Refusal.NONE;
    }

    /** Keeps track of whether the up cell's tags are currently in view (used to pause the feeder on a tip). */
    private void trackCluster(Inputs in) {
        if (cell == null || !in.frame.usable() || !in.frame.newFrame) {
            return;
        }
        if (upViewFor(in) != null) {
            lastUpClusterSeen = in.nowSec;
            everSawCluster = true;
            unseenSince = Double.NaN;
        }
    }

    /**
     * If the tracker now believes a different cell is up, drops back to planning for it (keeping the
     * shots already fired); if the tags of the believed cell have simply never been seen while standing
     * in place, assumes the other cell is up (once), then gives up.
     */
    private boolean retargetIfHiveChanged(Inputs in, Outputs out) {
        if (cell != null && tracker.upSide() != targetSide) {
            out.feederOn = false;
            if (pressShots >= cfg.maxShots) {
                enter(State.DONE, in.nowSec);
                return true;
            }
            enter(State.PLAN, in.nowSec);
            ready.reset();
            plan(in, out);
            return true;
        }
        boolean standing = state == State.ALIGN || state == State.FEED;
        if (standing && cfg.requireHiveConfirmation && in.frame.usable() && !everSawCluster && !tracker.confirmed()) {
            if (Double.isNaN(unseenSince)) {
                unseenSince = in.nowSec;
            }
            if (in.nowSec - unseenSince >= cfg.hiveUnseenFlipSec) {
                if (unseenFlips >= cfg.maxUnseenFlips) {
                    fail(Fault.HIVE_NOT_VISIBLE, in, out);
                    return true;
                }
                unseenFlips++;
                unseenSince = Double.NaN;
                tracker.flipBecauseUnseen();
                out.feederOn = false;
                enter(State.PLAN, in.nowSec);
                plan(in, out);
                return true;
            }
        }
        return false;
    }

    // ------------------------------------------------------------------------------------------ helpers

    private double odometryRange(Inputs in) {
        return ShotGeometry.rangeTo(in.pose, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
    }

    private double bestRange(Inputs in) {
        ClusterView v = upViewFor(in);
        if (v != null && v.hasGeometry && in.frame.usable()) {
            return v.rangeIn;
        }
        return odometryRange(in);
    }

    private void fail(Fault why, Inputs in, Outputs out) {
        fault = why;
        enter(State.FAULT, in.nowSec);
        out.feederOn = false;
        out.driveMode = DriveMode.MANUAL;
        out.waypoints = Collections.emptyList();
        out.flywheelTargetTps = cfg.idleTps;
    }

    private Outputs finish(Outputs out, Inputs in) {
        out.state = state;
        out.fault = fault;
        out.shotsFired = pressShots;
        out.targetSide = targetSide;
        out.cue = pendingCue;
        pendingCue = Cue.NONE;
        if (cell != null && in.pose != null) {
            out.rangeIn = Double.isNaN(out.rangeIn) ? odometryRange(in) : out.rangeIn;
            out.aimErrorDeg = Math.toDegrees(ShotGeometry.aimError(in.pose, cfg.launchForwardIn,
                    cfg.launchLeftIn, cell.originX, cell.originY));
        }
        if (state != State.FEED) {
            out.feederOn = false;                 // belt and braces: the feeder can only run in FEED
        }
        out.message = describe(out);
        return out;
    }

    private String describe(Outputs out) {
        switch (state) {
            case IDLE:
                return "Ready";
            case PLAN:
                return "Planning";
            case APPROACH:
                return "Driving to the scoring pose";
            case ALIGN:
                return "Aligning";
            case FEED:
                return out.refusal == Refusal.NONE ? "Feeding" : "Waiting: " + out.refusal;
            case DONE:
                return "Done: " + pressShots + " shot(s)";
            case FAULT:
                return "Stopped: " + fault;
            default:
                return "";
        }
    }
}
