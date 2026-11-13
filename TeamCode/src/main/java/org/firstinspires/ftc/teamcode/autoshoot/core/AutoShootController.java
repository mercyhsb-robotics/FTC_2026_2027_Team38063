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
 *       for the closest feasible pose. The flywheel starts spinning at once, for the range from where the robot
 *       stands now (that range converges on the planned one as the robot arrives).</li>
 *   <li><b>APPROACH</b> drives there (around the HIVE frame if needed) and hands over on MEASURED
 *       pose error, never on a path-finished flag.</li>
 *   <li><b>ALIGN</b> holds the pose; the camera trims the heading toward the aim point and the range
 *       toward the planned range; it waits for the robot to settle.</li>
 *   <li><b>FEED</b> runs the feeder only while a fail-closed gate ({@link Refusal}) says every
 *       condition is met, counts shots, and stops the moment the up cell's tags vanish (a tip).</li>
 * </ul>
 * Releasing the button returns to manual driving, feeder off, in the same loop.
 *
 * <p><b>If you are new to this file, read this first.</b> A STATE MACHINE is a program that is always in exactly one
 * named state (here IDLE, PLAN, APPROACH, ALIGN, FEED, DONE or FAULT). Every loop it looks at the newest
 * measurements, does the one job of its current state, and decides whether to switch to another state. The picture
 * above is the whole map. {@link #update} is the front door: it is called once per loop and dispatches to one small
 * method per state ({@code plan}, {@code approach}, {@code align}, {@code feed}). Every state method may fill in the
 * {@link Outputs} and may switch state with {@code enter(...)}.
 *
 * <p>Where it sits in the data flow (see docs/ARCHITECTURE.md): the TeleOp ({@code BioBuzzRobot.loop}) reads the
 * Pinpoint pose, the Limelight frame and the analyzed {@link ClusterView}s, fills an {@link Inputs}, calls
 * {@link #update}, and then applies the {@link Outputs}: flywheel speed and feeder go to the Launcher, the drive
 * mode goes to PedroDrive, and the {@link Cue} becomes a gamepad rumble. The {@link HiveTracker} is updated by the
 * TeleOp BEFORE this call; the controller only reads its belief ("which cell is UP?") and, rarely, tells it to flip.
 * Helpers it owns or uses: {@link ScoringPlanner} (where to stand), {@link ShotTable} (flywheel speed for a range),
 * {@link FlywheelReady} (is the flywheel at speed?), {@link ShotCounter} (how many balls have left?) and
 * {@link ShotGeometry} (range and aim math).
 *
 * <p>Words you will meet:
 * <ul>
 *   <li><b>Pose</b>: where the robot's rotation center is (x, y in inches) and which way it faces (heading in
 *       radians, counter-clockwise positive) in the Pedro field frame (origin at the corner where the audience wall
 *       meets the red wall, +x toward blue, +y away from the audience). <b>Odometry</b> is the pose measured by the
 *       wheels and the Pinpoint sensor, as opposed to what the camera says.</li>
 *   <li><b>Volley</b>: one burst of up to 4 shots from one button press (rule G407: a robot may hold at most
 *       4 POLLEN). <b>Tip</b>: when enough POLLEN lands, the HIVE tips and the OTHER cell becomes the UP one.</li>
 *   <li><b>Fail-closed</b>: when unsure, refuse to shoot. The feed gate ({@link Refusal}) lists everything that must
 *       be true before a ball may be fed, and the first one that is not true stops the feeder. This protects rule
 *       G417: launching at the outside of a cell, such as one that is tipping down, is a foul.</li>
 *   <li><b>Trim</b>: a small camera-based correction added to the planned pose. <b>Closed loop</b>: a controller that
 *       keeps measuring and correcting, as opposed to commanding something once and hoping.</li>
 *   <li><b>Heading wrap-around</b>: 179 degrees and -179 degrees are only 2 degrees apart, so angle differences go
 *       through {@link Angles} instead of plain subtraction.</li>
 *   <li><b>tps</b>: encoder ticks per second, the unit of flywheel speed. All times are seconds on the clock the
 *       caller supplies in {@link Inputs#nowSec}; speeds are inches per second; angles are radians unless the name
 *       says Deg.</li>
 * </ul>
 */
public final class AutoShootController {

    /**
     * Which step of the sequence the robot is in (see the picture above).
     * <ul>
     *   <li>IDLE: the shoot button is not being used; the driver has manual control.</li>
     *   <li>PLAN: choosing the cell and the scoring pose. Lasts less than one loop (it immediately moves on).</li>
     *   <li>APPROACH: driving to the scoring pose.</li>
     *   <li>ALIGN: holding the pose and settling, with small camera corrections.</li>
     *   <li>FEED: feeding POLLEN into the flywheel, as the gate allows.</li>
     *   <li>DONE: the volley has ended (all shots fired, the hopper ran empty, or the volley time limit passed);
     *       holds still until the button is released.</li>
     *   <li>FAULT: something went wrong ({@link Fault}); manual drive, shooter idle, until the button is released.</li>
     * </ul>
     */
    public enum State { IDLE, PLAN, APPROACH, ALIGN, FEED, DONE, FAULT }

    /**
     * Why the sequence gave up (reported on the Driver Station). Each fault puts the controller in FAULT, where it
     * hands control back to the driver and waits for the button to be released.
     * <ul>
     *   <li>NONE: no fault.</li>
     *   <li>NO_POSE: odometry has no trusted pose (the robot was never told where it is standing).</li>
     *   <li>NO_FEASIBLE_POSE: the planner found no legal place to shoot from (blocked or boxed in).</li>
     *   <li>APPROACH_TIMEOUT: driving took longer than distance / speed limit plus a fixed allowance.</li>
     *   <li>APPROACH_STALLED: the robot barely moved for a while although it is still far from the pose
     *       (blocked?).</li>
     *   <li>ALIGN_TIMEOUT: the robot never settled and is not even within the relaxed shot tolerances.</li>
     *   <li>SPINUP_TIMEOUT: feeding was never allowed within the spin-up allowance after FEED began
     *       (normally the flywheel never reached speed; any other refusal that never clears ends here too).</li>
     *   <li>DRIVER_OVERRIDE: the driver pushed the translation stick hard, which cancels the automatic drive.</li>
     *   <li>HIVE_NOT_VISIBLE: the tags of the believed UP cell were never seen, even after flipping to the other
     *       cell once.</li>
     *   <li>HIVE_UNCONFIRMED: declared but not raised by the current code (an unconfirmed HIVE only pauses feeding,
     *       see {@link Refusal#HIVE_UNCONFIRMED}).</li>
     *   <li>FEED_TIMEOUT: in DIP counting mode the whole volley time ran out without one counted shot.</li>
     * </ul>
     */
    public enum Fault {
        NONE, NO_POSE, NO_FEASIBLE_POSE, APPROACH_TIMEOUT, APPROACH_STALLED, ALIGN_TIMEOUT, SPINUP_TIMEOUT,
        DRIVER_OVERRIDE, HIVE_NOT_VISIBLE, HIVE_UNCONFIRMED, FEED_TIMEOUT
    }

    /**
     * The first reason the shot gate is closed, in priority order (permanent causes before transient ones).
     * "Gate" = a checklist that must all be true before the feeder may push a ball. Only the first failing item is
     * reported, so the driver sees the most fundamental problem. The feeder runs only when this is NONE.
     * <ul>
     *   <li>NONE: every condition holds; feeding is allowed.</li>
     *   <li>POSE_UNTRUSTED: odometry is not trusted, so we do not know where the robot is.</li>
     *   <li>HIVE_UNCONFIRMED: the camera (or the driver) has not yet confirmed which cell is UP.</li>
     *   <li>TAGS_LOST: the UP cell's tags were seen but have vanished for too long: the cell may be tipping
     *       (G417).</li>
     *   <li>OUT_OF_RANGE: the range to the aim point is outside the measured shot table.</li>
     *   <li>ROBOT_MOVING: the robot is still translating or turning too fast.</li>
     *   <li>HEADING_ERROR: the shooter is not pointing at the aim point closely enough.</li>
     *   <li>FLYWHEEL_NOT_AT_SPEED: the flywheel is outside its speed window.</li>
     *   <li>VOLLEY_COMPLETE: the shot limit for this press has been reached (rule G407).</li>
     * </ul>
     */
    public enum Refusal {
        NONE, POSE_UNTRUSTED, HIVE_UNCONFIRMED, TAGS_LOST, OUT_OF_RANGE, ROBOT_MOVING, HEADING_ERROR,
        FLYWHEEL_NOT_AT_SPEED, VOLLEY_COMPLETE
    }

    /**
     * One-shot notifications for the driver (rumble), fired on state entry. The TeleOp turns each into a gamepad
     * vibration. A cue is delivered in exactly one {@link Outputs}, then cleared.
     * <ul>
     *   <li>STARTED: meant to say "the press was accepted" (entering PLAN). As the code stands, PLAN immediately
     *       switches to APPROACH, ALIGN or FAULT in the same loop, which replaces the cue, so STARTED never actually
     *       reaches the driver.</li>
     *   <li>ARRIVED: the robot reached the scoring pose (entering ALIGN).</li>
     *   <li>DONE: the volley finished.</li>
     *   <li>FAULT: the sequence gave up.</li>
     * </ul>
     */
    public enum Cue { NONE, STARTED, ARRIVED, DONE, FAULT }

    /**
     * Who drives this loop, and how.
     * <ul>
     *   <li>MANUAL: the driver's sticks.</li>
     *   <li>FOLLOW: drive along {@link Outputs#waypoints}, ending at {@link Outputs#holdPose}, turning gradually
     *       toward that pose's heading.</li>
     *   <li>HOLD: stay at {@link Outputs#holdPose}, correcting continuously (a closed loop on position and
     *       heading), so pushes and drift are fought.</li>
     * </ul>
     */
    public enum DriveMode { MANUAL, FOLLOW, HOLD }

    // ------------------------------------------------------------------------------------------

    /**
     * Every number the sequence uses; defaults come from {@link BioBuzzConstants}. The fields are public and
     * mutable so a unit test can change one value. Names are shortened versions of the constants (for example
     * {@code handoffIn} comes from {@code AutoShoot.APPROACH_HANDOFF_IN}); the comment above each group says what the
     * group controls and gives units: inches, degrees, seconds, inches per second ("InS"), degrees per second
     * ("DegS"), ticks per second ("tps").
     */
    public static final class Config {
        // APPROACH hand-over: the drive is "arrived enough" to start ALIGN when within handoffIn inches, handoffDeg
        // degrees of heading and handoffSpeed in/s of the scoring pose. approachTimeoutSpeed (in/s) and
        // approachTimeoutExtra (seconds) set the time allowed to arrive: distance / speed + extra.
        public double handoffIn, handoffDeg, handoffSpeed, approachTimeoutSpeed, approachTimeoutExtra;
        // Stall detection (the robot is stuck): after stallGraceSec seconds of acceleration time, being slower than
        // stallSpeed (in/s) for stallSec seconds while still more than stallMinDist inches from the pose is a stall.
        public double stallSpeed, stallSec, stallGraceSec, stallMinDist;
        // ALIGN "settled" test: position error under alignPosTol (in), heading error under alignHeadingTolDeg (deg),
        // speed under alignSpeedTol (in/s) and turn rate under alignOmegaTolDegS (deg/s), all held for alignSettleSec
        // seconds. If that has not happened after alignTimeoutSec seconds, ALIGN gives up (or shoots on relaxed
        // limits).
        public double alignPosTol, alignHeadingTolDeg, alignSpeedTol, alignOmegaTolDegS, alignSettleSec, alignTimeoutSec;
        // If the robot is pushed more than this many inches from its hold pose, it drives back to it. If it is turned more than
        // reapproachHeadingDeg degrees away from the hold heading, it plans a new (wall-checked) route too: the drive would otherwise
        // spin it back in place, which swings a corner through a wall when one is close.
        public double reapproachIn, reapproachHeadingDeg;
        // Master switch for the camera trims (small heading and range corrections applied while in ALIGN).
        public boolean visionTrim;
        // Camera trim shape. Each update moves the hold heading by headingGain (a fraction, 0..1) of the camera's
        // aim error, but by at most headingStepDeg degrees, and the total heading trim never exceeds headingTrimMaxDeg.
        // The same for range with rangeGain, rangeStepIn and rangeTrimMaxIn (inches). trimPeriodSec is the minimum
        // time in seconds between two trims.
        public double headingGain, headingStepDeg, headingTrimMaxDeg, rangeGain, rangeStepIn, rangeTrimMaxIn, trimPeriodSec;
        // The shot gate's physical limits: translation under shotMaxSpeed (in/s), turn rate under shotMaxOmegaDegS
        // (deg/s), shooter pointing within shotHeadingTolDeg degrees of the aim point. shotRelaxedPosTol (in) is the
        // looser position limit allowed when ALIGN has timed out. spinupTimeoutSec: how long FEED may wait for the
        // flywheel before the volley is abandoned.
        public double shotMaxSpeed, shotMaxOmegaDegS, shotHeadingTolDeg, shotRelaxedPosTol, spinupTimeoutSec;
        // visionLossPauseSec: how long (seconds) the UP cell's tags may be missing before the feeder pauses.
        // hiveUnseenFlipSec: how long (seconds) to wait at the pose without ever seeing the tags before assuming the
        // other cell is up.
        public double visionLossPauseSec, hiveUnseenFlipSec;
        // If true, never feed until the camera (or the driver) has confirmed which cell is UP.
        public boolean requireHiveConfirmation;
        // How many times one press may flip to the other cell because the tags were never seen.
        public int maxUnseenFlips;
        // Driver override: the translation stick (0..1) held beyond overrideStick for overrideSec seconds cancels
        // the automatic drive.
        public double overrideStick, overrideSec;
        // launchForwardIn / launchLeftIn: where the ball leaves the robot, in inches from the rotation center in the
        // robot's own frame (forward is negative because the shooter is at the back; left is positive toward the
        // robot's left side). rangeMarginIn:
        // table-edge slack in inches (copied for completeness; this class does not read it). idleTps: flywheel speed
        // (tps) when nothing is being shot.
        public double launchForwardIn, launchLeftIn, rangeMarginIn, idleTps;
        // Shot limit for one press (rule G407: at most 4).
        public int maxShots;
        // How shots are counted: TIMED (by feeder time), DIP (by flywheel speed dips) or HYBRID (dips plus timeouts).
        public BioBuzzConstants.Shooter.ShotCountMode countMode;
        // How the feeder follows the flywheel: CONTINUOUS (keep running once started) or GATED_PER_BALL (only while
        // the gate is open).
        public BioBuzzConstants.Shooter.FeedMode feedMode;
        // Feeding time limits, in seconds: feedSecondsPerShot is the feeder time allowed per ball, volleyMaxSec caps
        // the whole volley, emptyAfterLastShotSec means "no ball seen for this long while feeding: the hopper is
        // empty".
        public double feedSecondsPerShot, volleyMaxSec, emptyAfterLastShotSec;
        // Flywheel "at speed" window and shot-dip detection (see FlywheelReady and ShotCounter): atBelow / atAbove are
        // fractions of the target speed, atFloor is extra slack in tps, atRise / atFall are the debounce times in
        // seconds, dipFrac is the speed drop (fraction of target) that counts as a ball leaving, refractory is the
        // quiet time in seconds after a counted ball.
        public double atBelow, atAbove, atFloor, atRise, atFall, dipFrac, refractory;
        // maxLoopDtSec: the longest time step (seconds) this class will add up for one loop (only the feeder-on timer
        // uses the clamped step). trimEnterPosFactor: camera trims start only within this many alignPosTol of the hold
        // position; trimEnterHeadingDeg: ... and within this many degrees of the hold heading.
        public double maxLoopDtSec, trimEnterPosFactor, trimEnterHeadingDeg;

        /** A Config holding the robot's real values, copied from {@link BioBuzzConstants}. */
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
            c.reapproachHeadingDeg = BioBuzzConstants.AutoShoot.REAPPROACH_HEADING_DEG;
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

    /**
     * Everything measured this loop. Build a fresh one every loop. Units: seconds, inches, inches per second,
     * radians per second, ticks per second.
     */
    public static final class Inputs {
        /** The logic clock in seconds. Any steady clock works; only differences between loops matter. */
        public double nowSec;
        /** True while the shoot button is held down. */
        public boolean shootHeld;
        /** The robot's pose from odometry (field frame, see {@link Pose2}). Null if there is no odometry. */
        public Pose2 pose;
        /** Translation speed in inches per second. */
        public double speedInS;
        /** Turn rate in radians per second (absolute value is used). */
        public double omegaRadS;
        /** True once the odometry has been given a trustworthy starting pose. */
        public boolean poseTrusted;
        /** Measured flywheel speed in ticks per second. */
        public double flywheelTps;
        /** This loop's camera snapshot; the default means "no camera connected". */
        public VisionFrame frame = VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, 0.0);
        /** {@link ClusterAnalyzer} output for this frame (own alliance only). */
        public List<ClusterView> views = Collections.emptyList();
        /** Magnitude (0-1) of the driver's translation stick. */
        public double stickMagnitude;
    }

    /**
     * Everything to command. Defaults are the safe ones: manual drive, flywheel idle, feeder off. The state methods
     * only override what they need, so anything not set stays safe.
     */
    public static final class Outputs {
        /** Who drives this loop (see {@link DriveMode}). */
        public DriveMode driveMode = DriveMode.MANUAL;
        /** For FOLLOW: the points to drive through; the last is {@link #holdPose}'s position (x, y in inches). */
        public List<double[]> waypoints = Collections.emptyList();
        /**
         * For FOLLOW: keep the heading the robot had at the start for this many inches of the route before turning
         * (0 = start turning at once). The route was checked against the walls with this exact turn, so the drive
         * must use it; see {@link HeadingProfile}.
         */
        public double turnHoldIn;
        /** For FOLLOW: be finished turning to {@link #holdPose}'s heading this many inches before the route ends. */
        public double turnSettleIn;
        /** For FOLLOW (the destination) and HOLD (the pose to hold). Only meaningful in those two modes. */
        public Pose2 holdPose;
        /** Flywheel speed to command, in ticks per second. */
        public double flywheelTargetTps;
        /** True to run the feeder (push a ball into the flywheel). Only ever true in FEED. */
        public boolean feederOn;
        /** A one-time driver notification this loop (or NONE). */
        public Cue cue = Cue.NONE;
        // Information for telemetry.
        /** The state after this loop's update. */
        public State state = State.IDLE;
        /** The fault that stopped the sequence (NONE if none). */
        public Fault fault = Fault.NONE;
        /** In FEED, the first closed gate condition (NONE means the gate is open). */
        public Refusal refusal = Refusal.NONE;
        /** Shots fired since the button was pressed (counted across a re-plan to the other cell). */
        public int shotsFired;
        /** The cell side the sequence is currently aiming at (null before the first PLAN). */
        public Side targetSide;
        /** Range from the ball exit to the aim point, in inches (NaN until a target cell is chosen). */
        public double rangeIn = Double.NaN;
        /** Turn still needed to point the shooter at the aim point, degrees, counter-clockwise positive (NaN: none). */
        public double aimErrorDeg = Double.NaN;
        /** A short human-readable status line for the Driver Station. */
        public String message = "";
    }

    // ------------------------------------------------------------------------------------------

    // Who we are shooting for, and the helpers we use. All are fixed for the life of the controller.
    private final Alliance alliance;
    private final FieldModel field;
    private final ScoringPlanner planner;
    private final ShotTable table;
    private final HiveTracker tracker;
    private final Config cfg;
    private final FlywheelReady ready;
    private final ShotCounter counter;

    // The state machine's current state, why it faulted (if it did), and the button-edge bookkeeping.
    private State state = State.IDLE;
    private Fault fault = Fault.NONE;
    private boolean prevHeld;
    /** False until the button has been seen released once, so holding it through INIT cannot auto-start a volley. */
    private boolean releasedSeen = false;
    // Time of the previous update() call (NaN before the first call).
    private double lastNow = Double.NaN;
    // Seconds since the previous update(), clamped to 0..maxLoopDtSec. Only the feeder-on timer adds this up.
    private double dt;
    // Time the current state was entered.
    private double stateSince;

    // The current target: which side's cell, the cell itself, and the planner's answer for it.
    private Side targetSide;
    private CellSpec cell;
    private ScoringPlanner.Solution solution;
    // Camera trims added to the planned hold pose: a heading change (radians, counter-clockwise positive) and a range
    // change (inches; positive moves the robot toward the HIVE).
    private double headingTrimRad;
    private double rangeTrimIn;
    // Time of the last trim, to rate-limit trims to one per trimPeriodSec.
    private double lastTrimSec = Double.NEGATIVE_INFINITY;
    // Seconds allowed for the current approach; NaN timers below mean "that condition is not currently true".
    private double approachTimeout;
    private double stallSince = Double.NaN;
    private double settledSince = Double.NaN;

    // Feeding bookkeeping for the current FEED segment (a press can have several segments if the HIVE tips).
    // shotsBase: shots already fired before this segment started. feedStart: when this FEED began. feederOnSec: how
    // long the feeder has actually run in it. feedLatched: CONTINUOUS mode only, true once the gate has opened.
    // frozenTps / frozenRange: the flywheel speed and range locked in when FEED began.
    private int shotsBase;
    private double feedStart;
    private double feederOnSec;
    private boolean feedLatched;
    private double frozenTps;
    private double frozenRange;

    // Tag-visibility bookkeeping: how many times we flipped to the other cell because the tags never showed up,
    // since when we have been standing without seeing them, when the UP cell's tags were last seen, whether they were
    // ever seen in this press, and since when the driver has been pushing the stick hard.
    private int unseenFlips;
    private double unseenSince = Double.NaN;
    private double lastUpClusterSeen = Double.NEGATIVE_INFINITY;
    private boolean everSawCluster;
    private double overrideSince = Double.NaN;

    // Shots fired since the button was pressed (all FEED segments together).
    private int pressShots;

    /**
     * @param alliance which alliance's HIVE we shoot into (cells are looked up with it)
     * @param field    the field map (cells and obstacles)
     * @param planner  chooses the scoring pose
     * @param table    flywheel speed as a function of range
     * @param tracker  the robot's belief about which cell is UP; owned and updated by the TeleOp
     * @param cfg      all tunable numbers (normally {@link Config#fromConstants()})
     */
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

    /** The current state (the TeleOp uses it, for example, to skip drift correction while feeding). */
    public State state() {
        return state;
    }

    /** The reason for the last FAULT, or NONE. Cleared when the button is released. */
    public Fault fault() {
        return fault;
    }

    /** Shots fired since the button was pressed. */
    public int shotsFired() {
        return pressShots;
    }

    /** The planner's latest answer for this press (null before the first PLAN or after a release). */
    public ScoringPlanner.Solution solution() {
        return solution;
    }

    // ------------------------------------------------------------------------------------------

    /**
     * Runs one loop of the state machine and returns what to command. Call it exactly once per OpMode loop, every
     * loop, even while the button is not held (it needs to see the release). The returned {@link Outputs} is new
     * each time.
     */
    public Outputs update(Inputs in) {
        // 1. How much time passed since the last loop? Clamped so one very long loop (a hiccup) cannot make the
        //    feeder timer jump ahead; the first call has no previous loop, so it counts as zero.
        dt = Double.isNaN(lastNow) ? 0.0 : Math.max(0.0, Math.min(cfg.maxLoopDtSec, in.nowSec - lastNow));
        lastNow = in.nowSec;

        // 2. A "rising edge" is the loop in which the button goes from up to down: that is a fresh press.
        boolean rising = in.shootHeld && !prevHeld;
        prevHeld = in.shootHeld;

        Outputs out = new Outputs();
        out.flywheelTargetTps = cfg.idleTps;

        // 3. Button released: stop everything this very loop. Outputs stay at the safe defaults (manual drive,
        //    feeder off, flywheel idle) and the state machine goes back to IDLE, forgetting this press.
        if (!in.shootHeld) {
            releasedSeen = true;
            if (state != State.IDLE) {
                enter(State.IDLE, in.nowSec);
                fault = Fault.NONE;
                resetPress();
            }
            return finish(out, in);
        }

        // 4. Waiting in IDLE with the button held: start only on a fresh press that came after a release. That way
        //    holding the button through INIT, or after a finished volley, does not start (or restart) a volley.
        if (state == State.IDLE) {
            if (rising && releasedSeen) {
                releasedSeen = false;
                begin(in);
            } else {
                return finish(out, in);
            }
        }

        // 5. Safety checks that apply to every active state.
        if (state != State.DONE && state != State.FAULT) {
            // Without a trusted pose the robot does not know where it is, so driving or shooting would be blind.
            if (!in.poseTrusted || in.pose == null) {
                fail(Fault.NO_POSE, in, out);
                return finish(out, in);
            }
            // Driver override: pushing the translation stick hard for a moment means "I want to drive". Requiring
            // overrideSec seconds of it ignores a brief bump of the stick.
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
            // Remember whether the UP cell's tags are in view (used later to pause the feeder if they vanish).
            trackCluster(in);
        }

        // 6. Do the one job of the current state.
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
                // Finished: hold still where we are (not at the old target) and wait for the release.
                out.driveMode = DriveMode.HOLD;
                out.holdPose = in.pose;
                break;
            case FAULT:
            default:
                // Stay in the safe defaults: manual drive, feeder off, flywheel idle.
                break;
        }
        return finish(out, in);
    }

    // ------------------------------------------------------------------------------------------ states

    /** A fresh press was accepted: forget the previous press and go and plan. */
    private void begin(Inputs in) {
        resetPress();
        enter(State.PLAN, in.nowSec);
    }

    /** Clears everything that belongs to one button press (shots, trims, flips, the chosen cell and plan). */
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

    /**
     * Switches state: remembers when it began, clears the per-state timers (stall and settle) and queues the
     * driver cue that goes with the new state.
     */
    private void enter(State next, double now) {
        state = next;
        stateSince = now;
        stallSince = Double.NaN;
        settledSince = Double.NaN;
        pendingCue = cueFor(next);
    }

    // The cue waiting to be delivered: enter() sets it, finish() hands it to the driver and clears it. If a state is
    // entered and left within one loop, only the last state's cue survives.
    private Cue pendingCue = Cue.NONE;

    /** The driver cue that goes with entering a state; only PLAN, ALIGN, DONE and FAULT have one. */
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

    /**
     * PLAN: pick the cell the tracker says is UP, ask the planner for the closest feasible pose, and move on to
     * APPROACH (or straight to ALIGN if the robot is already there). Also used to re-plan after the HIVE tips.
     */
    private void plan(Inputs in, Outputs out) {
        targetSide = tracker.upSide();
        cell = field.cell(alliance, targetSide);
        solution = planner.solve(in.pose, cell);
        if (!solution.ok()) {
            fail(Fault.NO_FEASIBLE_POSE, in, out);
            return;
        }
        // A new plan starts with no camera trims (they belonged to the old pose).
        headingTrimRad = 0.0;
        rangeTrimIn = 0.0;
        // Allow a time proportional to the route length, plus a fixed allowance for starting and stopping.
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

    /**
     * True if the robot is already close enough to the planned pose to start the fine alignment: position, heading
     * and speed all within the hand-over limits. This is MEASURED error (from odometry), on purpose: Pedro's own
     * "path finished" flag does not mean the robot arrived.
     */
    private boolean arrivedNear(Inputs in) {
        double dist = in.pose.distanceTo(solution.pose);
        // Angles.distance wraps around, so 179 degrees vs -179 degrees counts as 2 degrees apart.
        double headingErr = Math.toDegrees(Angles.distance(in.pose.heading, solution.pose.heading));
        return dist <= cfg.handoffIn && headingErr <= cfg.handoffDeg && in.speedInS <= cfg.handoffSpeed;
    }

    /** APPROACH: follow the planned route to the scoring pose, and watch for arrival, a timeout or a stall. */
    private void approach(Inputs in, Outputs out) {
        if (retargetIfHiveChanged(in, out)) {
            return;
        }
        // Ask the drive to follow the route; the last waypoint is the pose's position and its heading is the final
        // heading to turn to.
        out.driveMode = DriveMode.FOLLOW;
        out.waypoints = solution.waypoints;
        // The planner checked this route against the walls with this exact turn (when the robot may start turning and
        // when it must be done), so the drive must be given the same window.
        out.turnHoldIn = solution.turnHoldIn;
        out.turnSettleIn = solution.turnSettleIn;
        out.holdPose = solution.pose;
        // Spin the flywheel up on the way, for the range from where the robot is right now, so it is close to speed
        // on arrival. While the robot is far away that range is outside the shot table and velocityAt() clamps to
        // the nearest table row.
        out.flywheelTargetTps = table.velocityAt(odometryRange(in));

        double elapsed = in.nowSec - stateSince;
        double dist = in.pose.distanceTo(solution.pose);
        // Arrived: hand over to ALIGN, which does the precise holding and settling.
        if (arrivedNear(in)) {
            enter(State.ALIGN, in.nowSec);
            align(in, out);
            return;
        }
        // Took too long.
        if (elapsed > approachTimeout) {
            fail(Fault.APPROACH_TIMEOUT, in, out);
            return;
        }
        // Stall: after the start-up grace period, barely moving while still far from the pose, for long enough,
        // means the robot is blocked (a wall, another robot) rather than slowly arriving.
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

    /**
     * The pose ALIGN and FEED hold: the planned pose plus the camera trims. The heading trim turns the robot; the
     * range trim slides it along its shooting direction.
     */
    private Pose2 holdPose() {
        Pose2 base = solution.pose;
        double h = Angles.normalize(base.heading + headingTrimRad);
        // The shooter fires out the back, so "toward the HIVE" is backward: a positive range trim moves the robot
        // backward along its heading and shortens the range.
        double x = base.x - rangeTrimIn * Math.cos(base.heading);     // toward the target = along the shooter = backward
        double y = base.y - rangeTrimIn * Math.sin(base.heading);
        return new Pose2(x, y, h);
    }

    /**
     * ALIGN: hold the pose (closed loop), let the camera trim it, and wait until the robot has been settled for long
     * enough, then start feeding. Shooting while still swaying would miss, so settling is required.
     */
    private void align(Inputs in, Outputs out) {
        if (retargetIfHiveChanged(in, out)) {
            return;
        }
        Pose2 hold = holdPose();
        out.driveMode = DriveMode.HOLD;
        out.holdPose = hold;
        // Use the camera's range if available (it measures the real distance), otherwise the odometry range.
        out.flywheelTargetTps = table.velocityAt(bestRange(in));

        // How far the robot is from where it should be, in inches, degrees and degrees per second.
        double posErr = in.pose.distanceTo(hold);
        double headErr = Math.toDegrees(Angles.distance(in.pose.heading, hold.heading));
        double omegaDegS = Math.toDegrees(Math.abs(in.omegaRadS));

        // Pushed far away (for example by another robot): drive back to the pose first. This plans a NEW route from
        // where the robot is now. Re-using the old route would send the robot from here to the old route's first
        // point along a line nobody checked against the walls and the HIVE. The same for a robot that was only TURNED a long way (its
        // position is fine): holding the pose would spin it back in place, which swings a corner through a wall when one is close, and
        // only a planned route is checked for that (the plan drives out to open floor first when there is no room to turn).
        if (posErr > cfg.reapproachIn || headErr > cfg.reapproachHeadingDeg) {
            plan(in, out);
            return;
        }

        // Camera trims only start once the robot is already close to its hold pose (within trimEnterPosFactor
        // alignment tolerances and trimEnterHeadingDeg degrees), so the camera only fine-tunes a pose the robot has
        // nearly reached. After a trim the hold pose has moved, so the errors are measured again against the new one.
        boolean nearHold = posErr <= cfg.trimEnterPosFactor * cfg.alignPosTol && headErr <= cfg.trimEnterHeadingDeg;
        if (cfg.visionTrim && nearHold) {
            applyVisionTrim(in);
            hold = holdPose();
            out.holdPose = hold;
            posErr = in.pose.distanceTo(hold);
            headErr = Math.toDegrees(Angles.distance(in.pose.heading, hold.heading));
        }

        // Settled = position, heading, speed and turn rate are ALL within tolerance. It must stay that way for
        // alignSettleSec seconds in a row (settledSince restarts the count whenever anything slips out of tolerance).
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
            // Never settled perfectly (a mecanum drive rarely settles exactly). If the robot is still close enough to
            // the pose and points within the shot tolerance, shoot anyway; otherwise give up.
            double aimErrDeg = Math.toDegrees(Math.abs(ShotGeometry.aimError(in.pose, cfg.launchForwardIn,
                    cfg.launchLeftIn, cell.originX, cell.originY)));
            if (posErr <= cfg.shotRelaxedPosTol && aimErrDeg <= cfg.shotHeadingTolDeg) {
                startFeed(in);
            } else {
                fail(Fault.ALIGN_TIMEOUT, in, out);
            }
        }
    }

    /**
     * Lets the camera nudge the hold heading toward the aim point and the hold range toward the planned range.
     * Each nudge is a fraction (the gain) of the error, limited per update and in total, and rate-limited in time,
     * so a noisy reading can only move the robot a little.
     */
    private void applyVisionTrim(Inputs in) {
        // Only on a brand-new, trustworthy camera frame, and not more often than once per trimPeriodSec.
        if (!in.frame.usable() || !in.frame.newFrame || in.nowSec - lastTrimSec < cfg.trimPeriodSec) {
            return;
        }
        ClusterView best = upViewFor(in);
        if (best == null) {
            return;
        }
        lastTrimSec = in.nowSec;
        // Heading: turn toward where the camera says the aim point is (aimErrorRad > 0 means turn counter-clockwise).
        if (best.hasBearing) {
            double step = Math.toRadians(cfg.headingStepDeg);
            double delta = Math.max(-step, Math.min(step, cfg.headingGain * best.aimErrorRad));
            headingTrimRad += delta;
            double max = Math.toRadians(cfg.headingTrimMaxDeg);
            headingTrimRad = Math.max(-max, Math.min(max, headingTrimRad));
        }
        // Range: if the camera measures a longer range than planned, trim positive (move toward the HIVE), and
        // the reverse.
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

    /** This frame's camera view of the cell we are aiming at, but only if it reads UP; otherwise null. */
    private ClusterView upViewFor(Inputs in) {
        for (ClusterView v : in.views) {
            // == is fine here: FieldModel owns exactly one CellSpec object per cell.
            if (v.cell == cell && v.orientation == ClusterView.Orientation.UP) {
                return v;
            }
        }
        return null;
    }

    /**
     * ALIGN is done: switch to FEED. This locks in the range and flywheel speed (so they stay steady during the volley)
     * and starts the shot counter for this feeding segment.
     */
    private void startFeed(Inputs in) {
        enter(State.FEED, in.nowSec);
        feedStart = in.nowSec;
        feederOnSec = 0.0;
        feedLatched = false;
        // Freeze the range from the camera if it is trustworthy, otherwise from odometry; the flywheel speed follows.
        // Frozen on purpose: a flywheel target that wiggled with every camera reading would keep leaving the speed
        // window and confuse the shot counter.
        ClusterView v = upViewFor(in);
        frozenRange = (v != null && v.hasGeometry && in.frame.usable()) ? v.rangeIn : odometryRange(in);
        frozenTps = table.velocityAt(frozenRange);
        counter.start(in.nowSec);
        // Shots from earlier segments of this press (before a re-plan) stay counted.
        shotsBase = pressShots;
    }

    /**
     * FEED: hold the pose, run the feeder only while the gate is open, count the balls, and finish (DONE) when the
     * volley is complete, empty or out of time. Rule G407 limits the volley to 4 shots.
     */
    private void feed(Inputs in, Outputs out) {
        if (retargetIfHiveChanged(in, out)) {
            return;
        }
        // 1. Keep holding the pose, with the locked flywheel speed and range.
        Pose2 hold = holdPose();
        out.driveMode = DriveMode.HOLD;
        out.holdPose = hold;
        out.flywheelTargetTps = frozenTps;
        out.rangeIn = frozenRange;

        // Pushed out of place, or turned a long way: stop feeding and align again (ALIGN plans a new route for either).
        if (in.pose.distanceTo(hold) > cfg.reapproachIn
                || Math.toDegrees(Angles.distance(in.pose.heading, hold.heading)) > cfg.reapproachHeadingDeg) {
            enter(State.ALIGN, in.nowSec);
            align(in, out);
            return;
        }

        // 2. Is the flywheel at speed, and does the gate allow a ball right now?
        boolean atSpeed = ready.update(in.nowSec, true, frozenTps, in.flywheelTps);
        Refusal refusal = evaluateGate(in, atSpeed);
        out.refusal = refusal;

        // 3. Decide the feeder. GATED_PER_BALL: on exactly while the whole gate is open. CONTINUOUS: once the gate has
        //    opened the feeder keeps running (a dip in flywheel speed is expected as each ball passes, so it does not
        //    stop for that), but any other refusal, such as lost tags or a moving robot, stops it at once.
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

        // 4. Time the feeder has run, and watch the flywheel for the speed dip each ball causes.
        if (out.feederOn) {
            feederOnSec += dt;
        }
        counter.update(in.nowSec, out.feederOn, atSpeed, frozenTps, in.flywheelTps);

        // 5. How many shots so far? "remaining" = shots still allowed (the cap minus those from earlier segments).
        //    TIMED mode counts one shot per feedSecondsPerShot of feeder time (the 1e-9 forgives rounding); DIP and
        //    HYBRID count the dips seen by the counter. Never more than the remaining allowance.
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

        // 6. Finish conditions, most important first.
        double sinceFeedStart = in.nowSec - feedStart;
        // Full volley (rule G407): stop feeding and hold still.
        if (pressShots >= cfg.maxShots) {
            enter(State.DONE, in.nowSec);
            out.feederOn = false;
            out.driveMode = DriveMode.HOLD;
            out.holdPose = in.pose;
            return;
        }
        // TIMED mode: enough feeder time has passed for the remaining balls.
        if (cfg.countMode == BioBuzzConstants.Shooter.ShotCountMode.TIMED && timed >= remaining) {
            enter(State.DONE, in.nowSec);
            out.feederOn = false;
            return;
        }
        if (cfg.countMode == BioBuzzConstants.Shooter.ShotCountMode.HYBRID) {
            // Hopper empty: at least one ball was counted, the feeder is running, yet no new ball has dipped the
            // flywheel for a while, so nothing is left to shoot. Dips missed: the feeder has run long enough for one
            // MORE ball than we still expect, so the dip counter must have missed some; stop rather than run forever.
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
        // Absolute time limit for the whole volley (plus the spin-up allowance). In DIP mode with no counted shot we
        // cannot tell whether anything was fired, so that is a fault; otherwise just finish.
        if (sinceFeedStart > cfg.volleyMaxSec + cfg.spinupTimeoutSec) {
            if (cfg.countMode == BioBuzzConstants.Shooter.ShotCountMode.DIP && dips == 0) {
                fail(Fault.FEED_TIMEOUT, in, out);
            } else {
                enter(State.DONE, in.nowSec);
                out.feederOn = false;
            }
        }
    }

    /**
     * The fail-closed gate: the first reason a ball may NOT be fed right now, or NONE. The checks run in a fixed
     * order, permanent problems first, so the one reported is the most basic thing to fix.
     */
    private Refusal evaluateGate(Inputs in, boolean atSpeed) {
        // We must know where the robot is.
        if (!in.poseTrusted) {
            return Refusal.POSE_UNTRUSTED;
        }
        // Rule G417: only launch into an UP cell. If a working camera has not yet confirmed which cell is up, or the
        // driver has not forced it, refuse. With no usable camera there is nothing to confirm with, so this check
        // is skipped and the driver's button is the only way to correct the belief.
        boolean cameraWorking = in.frame.usable();
        if (cfg.requireHiveConfirmation && cameraWorking && !tracker.confirmed()) {
            return Refusal.HIVE_UNCONFIRMED;
        }
        // The UP cell's tags were visible but have been gone for visionLossPauseSec: the cell is probably tipping, and
        // launching into a tipping cell is a foul (G417). Pause immediately; the tracker flips its belief a little
        // later. This also covers a camera that was working and then stopped delivering frames: we saw the tags, so
        // not seeing them any more must stop the feeder (fail closed). A robot that never saw the tags (no camera
        // fitted) is not affected.
        if (everSawCluster && in.nowSec - lastUpClusterSeen >= cfg.visionLossPauseSec) {
            return Refusal.TAGS_LOST;
        }
        // The shot table only covers a band of ranges; outside it the flywheel speed would be a guess.
        if (!table.inRange(frozenRange)) {
            return Refusal.OUT_OF_RANGE;
        }
        // A moving robot (still driving or turning) throws the ball off line.
        if (in.speedInS > cfg.shotMaxSpeed || Math.toDegrees(Math.abs(in.omegaRadS)) > cfg.shotMaxOmegaDegS) {
            return Refusal.ROBOT_MOVING;
        }
        // The shooter (the robot's back) must point at the aim point; checked from odometry geometry.
        double aimErr = Math.toDegrees(Math.abs(ShotGeometry.aimError(in.pose, cfg.launchForwardIn,
                cfg.launchLeftIn, cell.originX, cell.originY)));
        if (aimErr > cfg.shotHeadingTolDeg) {
            return Refusal.HEADING_ERROR;
        }
        // The flywheel must be inside its speed window (a slow ball falls short).
        if (!atSpeed) {
            return Refusal.FLYWHEEL_NOT_AT_SPEED;
        }
        // Rule G407: never more than the volley limit.
        if (pressShots >= cfg.maxShots) {
            return Refusal.VOLLEY_COMPLETE;
        }
        return Refusal.NONE;
    }

    /** Keeps track of whether the up cell's tags are currently in view (used to pause the feeder on a tip). */
    private void trackCluster(Inputs in) {
        // Only a NEW, usable camera frame can tell us anything; repeated or dead frames are ignored.
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
     *
     * @return true if it changed state (the caller must stop what it was doing); false to carry on
     */
    private boolean retargetIfHiveChanged(Inputs in, Outputs out) {
        // The belief changed (the camera saw the HIVE tip or its tags vanished, the driver pressed the toggle
        // button, or we flipped below). Stop feeding at once; if the shot limit is already reached we are done,
        // otherwise plan again for the other cell. pressShots is kept, so the volley still totals at most the limit
        // (G407).
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
        // Standing at the pose with a working camera, never having seen this cell's tags, and with the belief
        // unconfirmed: maybe the HIVE is tipped the other way. Wait hiveUnseenFlipSec seconds, then flip the belief
        // (at most maxUnseenFlips times per press) and plan for the other cell; after that, give up.
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

    /** Range in inches from the ball exit to the aim point, using only odometry (the pose and the launch point). */
    private double odometryRange(Inputs in) {
        return ShotGeometry.rangeTo(in.pose, cfg.launchForwardIn, cfg.launchLeftIn, cell.originX, cell.originY);
    }

    /** The best range available in inches: the camera's measurement when it is usable, else odometry. */
    private double bestRange(Inputs in) {
        ClusterView v = upViewFor(in);
        if (v != null && v.hasGeometry && in.frame.usable()) {
            return v.rangeIn;
        }
        return odometryRange(in);
    }

    /**
     * Gives up: records why, enters FAULT, and hands control back (manual drive, feeder off, no route, flywheel idle).
     * The controller stays in FAULT until the button is released, so a stuck button cannot restart anything.
     */
    private void fail(Fault why, Inputs in, Outputs out) {
        fault = why;
        enter(State.FAULT, in.nowSec);
        out.feederOn = false;
        out.driveMode = DriveMode.MANUAL;
        out.waypoints = Collections.emptyList();
        out.flywheelTargetTps = cfg.idleTps;
    }

    /**
     * The last step of every update(): copies the bookkeeping into the Outputs (state, fault, shots, target side,
     * range, aim error, the one-time cue and a status message) and applies the final safety rule for the feeder.
     */
    private Outputs finish(Outputs out, Inputs in) {
        out.state = state;
        out.fault = fault;
        out.shotsFired = pressShots;
        out.targetSide = targetSide;
        // Deliver the queued cue once, then clear it.
        out.cue = pendingCue;
        pendingCue = Cue.NONE;
        // Range and aim error for telemetry, whenever a target cell exists. The aim error is positive when the
        // shooter must turn counter-clockwise.
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

    /** A short status line for the Driver Station telemetry. */
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
