package org.firstinspires.ftc.teamcode.autoshoot.run;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.AutoShootController;
import org.firstinspires.ftc.teamcode.autoshoot.core.ClusterAnalyzer;
import org.firstinspires.ftc.teamcode.autoshoot.core.ClusterView;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.HiveTracker;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.PoseStore;
import org.firstinspires.ftc.teamcode.autoshoot.core.ScoringPlanner;
import org.firstinspires.ftc.teamcode.autoshoot.core.ShotTable;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.TipConfirmation;
import org.firstinspires.ftc.teamcode.autoshoot.core.TipWait;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.DrivePath;
import org.firstinspires.ftc.teamcode.autoshoot.routes.FlowerPress;
import org.firstinspires.ftc.teamcode.autoshoot.routes.FlowerWiggleSequence;
import org.firstinspires.ftc.teamcode.autoshoot.routes.IntakeSource;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteAnalysis;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Step;

/**
 * The ROUTE RUNNER: it takes a route that the team has already checked (an {@link AutoRoute} for one alliance, played through by
 * {@link RouteAnalysis}) and, one loop at a time, says what the robot must do to DRIVE it. It has the same shape as
 * {@code AutoShootController}: every loop the caller hands in what was measured ({@link Inputs}) and gets back what to command
 * ({@link Outputs}). Nothing here touches the FTC SDK, Pedro or a clock, so the whole run can be simulated and tested on a PC
 * ({@code RouteExecutorSimTest}); the hardware adapter only turns the answer into motor powers.
 *
 * <p><b>The rules it keeps</b> (the user's fourth constraint and the game rules):
 * <ul>
 *   <li><b>The camera decides, never the clock.</b> A volley at a cell that is UP only because the HIVE has tipped (the partner's tip,
 *       or one of ours: {@code StepInfo.tipDependent}) starts only after {@link TipConfirmation} says the camera has seen that cell UP,
 *       steady for the settle time. A WAIT_FOR_TIP is a {@link TipWait}: it ends when the camera confirms, or at its GIVE-UP DEADLINE on
 *       the route clock ({@code RouteAnalysis.effectiveGiveUpSec}: the step's own deadline or the latest the rest of the route allows).
 *       At the deadline every volley that needs the tip is skipped and the route goes on with the safe steps (LEAVE, PARK). No camera,
 *       no shot.</li>
 *   <li><b>At most 4 POLLEN (G407).</b> The runner counts what it holds. A FLOWER pick-up stops at 4 ({@link FlowerWiggleSequence}), a
 *       GARDEN or floor pick-up stops its intake when 4 are in, and a volley never fires more than is held.</li>
 *   <li><b>Fail closed.</b> A move that never arrives, a turn that never gets there, a volley that faults: the reason is recorded
 *       ({@link #failures()}), the step is given up and the route goes on. It never fires blind: the shooting itself is the
 *       {@code AutoShootController}, with the same gate as in TeleOp (pose, camera-confirmed cell, range, aim, flywheel speed).</li>
 *   <li><b>The route clock</b> stops everything at {@code Run.CLOCK_STOP_SEC}: the drive holds where it is, the flywheel is 0, the feeder
 *       and the intake are off.</li>
 * </ul>
 *
 * <p><b>How each step runs</b> (the driving uses the geometry the checker swept, never a second copy of a rule):
 * <ul>
 *   <li>START: the first loop sets the odometry to the start pose ({@link Outputs#setPose}).</li>
 *   <li>DRIVE: first a turn on the spot to the heading the leg starts with (when it is more than {@code Run.PRE_TURN_MIN_DEG}: that is the
 *       turn the checker swept against the walls), then the leg itself as a {@link DriveCommand.Kind#LEG}: the {@link DrivePath} the
 *       validator checked, built from where the robot really stands.</li>
 *   <li>SHOOT: a step with a pose first moves there; a volley that needs a confirmed tip waits at the pose for the camera; then the
 *       {@code AutoShootController} fires, as if the driver held the shoot button.</li>
 *   <li>INTAKE: GARDEN and FLOOR drive to the pose and run the intake for the dwell; a FLOWER lines up, drives in slowly, wiggles with
 *       {@link FlowerWiggleSequence} (the follower holds the position, only the target heading alternates) and backs out.</li>
 *   <li>WAIT, WAIT_FOR_TIP, PARK as their names say.</li>
 * </ul>
 *
 * <p>The poses of the plan are only used where they are exact (the FLOWER wiggle poses, the PARK pose, the pose of a SHOOT). A DRIVE leg
 * is built from the robot's measured pose when the leg begins, so a step that was skipped or a robot that is a little off cannot make the
 * runner drive a path nobody built.
 */
public final class RouteExecutor {

    /** What the runner is doing right now: for the screen and for the tests. */
    public enum Phase {
        START, TURNING, DRIVING, MOVING, WAITING, WAITING_FOR_TIP, CONFIRMING_TIP, SHOOTING, INTAKING,
        FLOWER_LINE_UP, FLOWER_APPROACH, FLOWER_WIGGLE, FLOWER_RETREAT, PARKING, FINISHED, STOPPED
    }

    /** Every number the runner uses; the defaults come from {@link BioBuzzConstants}. The fields are public so that a test can change one. */
    public static final class Config {
        /** The route time, in seconds, at which everything stops. */
        public double clockStopSec;
        /** The route time, in seconds, by which the route should be over (the AUTO limit minus the safety margin). */
        public double timeBudgetSec;
        /** Arrived: within this many inches, slower than {@link #arriveSpeedInS}. */
        public double arrivePosTolIn;
        public double arriveSpeedInS;
        /** A turn on the spot is done within this many degrees. */
        public double turnDoneDeg;
        /** Stopped within {@link #arriveLooseIn} of the end of a move for {@link #arriveLooseStillSec} counts as arrived. */
        public double arriveLooseIn;
        public double arriveLooseStillSec;
        /** A turn before a leg smaller than this many degrees is not stopped for. */
        public double preTurnMinDeg;
        /** A turn on the spot gives up after this many seconds. */
        public double turnTimeoutSec;
        /** A move gives up after distance / this speed (inches per second) plus {@link #moveTimeoutExtraSec}. */
        public double moveTimeoutSpeedInS;
        public double moveTimeoutExtraSec;
        /** The move to the pose of a SHOOT is "close enough to hand over" within these (the shooting sequence aligns the rest). */
        public double handoffIn;
        public double handoffDeg;
        public double handoffSpeedInS;
        /** The FLOWER pick-up: line-up and start tolerances (inches) and the fraction of cruise speed for the straight approach and retreat. */
        public double flowerLineUpTolIn;
        public double flowerStartTolIn;
        public double flowerSpeedFraction;
        /** One SHOOT step gives up after this many seconds. */
        public double shootStepTimeoutSec;
        /** The most POLLEN the robot may hold (G407). */
        public int capacity;
        /** The flywheel speed while nothing is being shot, in ticks per second. */
        public double idleFlywheelTps;

        /** The values in {@link BioBuzzConstants}. */
        public static Config fromConstants() {
            Config c = new Config();
            c.clockStopSec = BioBuzzConstants.Run.CLOCK_STOP_SEC;
            c.timeBudgetSec = RouteAnalysis.timeBudgetSec();
            c.arrivePosTolIn = BioBuzzConstants.Run.ARRIVE_POS_TOL_IN;
            c.arriveSpeedInS = BioBuzzConstants.Run.ARRIVE_SPEED_IN_S;
            c.turnDoneDeg = BioBuzzConstants.Run.TURN_DONE_DEG;
            c.arriveLooseIn = BioBuzzConstants.Run.ARRIVE_LOOSE_IN;
            c.arriveLooseStillSec = BioBuzzConstants.Run.ARRIVE_LOOSE_STILL_SEC;
            c.preTurnMinDeg = BioBuzzConstants.Run.PRE_TURN_MIN_DEG;
            c.turnTimeoutSec = BioBuzzConstants.Run.TURN_TIMEOUT_SEC;
            c.moveTimeoutSpeedInS = BioBuzzConstants.Run.MOVE_TIMEOUT_SPEED_IN_S;
            c.moveTimeoutExtraSec = BioBuzzConstants.AutoShoot.APPROACH_TIMEOUT_EXTRA_SEC;
            c.handoffIn = BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_IN;
            c.handoffDeg = BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_DEG;
            c.handoffSpeedInS = BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_SPEED_IN_S;
            c.flowerLineUpTolIn = BioBuzzConstants.Run.FLOWER_LINEUP_TOL_IN;
            c.flowerStartTolIn = BioBuzzConstants.Run.FLOWER_START_TOL_IN;
            c.flowerSpeedFraction = BioBuzzConstants.Auto.FLOWER_APPROACH_SPEED_FRACTION;
            c.shootStepTimeoutSec = BioBuzzConstants.Run.SHOOT_STEP_TIMEOUT_SEC;
            c.capacity = BioBuzzConstants.Auto.POLLEN_CAPACITY;
            c.idleFlywheelTps = BioBuzzConstants.Shooter.FLYWHEEL_IDLE_TPS;
            return c;
        }
    }

    /** Everything measured this loop. */
    public static final class Inputs {
        /** The route clock: seconds since AUTO started (any steady clock that starts at 0 when PLAY is pressed). */
        public double nowSec;
        /** Where odometry says the robot is (field frame, inches and radians). */
        public Pose2 pose;
        /** Translation speed, inches per second. */
        public double speedInS;
        /** Turn rate, radians per second. */
        public double omegaRadS;
        /** Measured flywheel speed, ticks per second. */
        public double flywheelTps;
        /** This loop's camera snapshot; the default means "no camera". */
        public VisionFrame frame = VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, 0.0);
    }

    /** Everything to command. The defaults are the safe ones: hold still, flywheel idle, feeder and intake off. */
    public static final class Outputs {
        /** Non-null on the first loop only: tell the odometry the robot stands here (the START step). */
        public Pose2 setPose;
        /** What the drive is to do. */
        public DriveCommand drive;
        /** Flywheel speed, ticks per second. */
        public double flywheelTps;
        /** True to run the feeder (push a POLLEN into the flywheel). */
        public boolean feederOn;
        /** True to run the intake motor and both corner servos. */
        public boolean intakeOn;
        /** True once the route is over (every step done, or the clock stopped it); the drive then only holds. */
        public boolean finished;
        /** The compact picture of where the run is, for the screen. */
        public Status status;
    }

    /** A compact picture of the run for the Driver Station screen. */
    public static final class Status {
        /** The step being run, counted from 1 (0 before the first loop). */
        public final int stepNumber;
        public final int stepCount;
        /** The step in words, for example "SHOOT 4 at UP cell from the solver's pose". */
        public final String stepText;
        /** What the robot is doing right now, in plain words. */
        public final String doing;
        public final Phase phase;
        /** POLLEN the robot holds (the plan's count, kept by the runner). */
        public final int pollenHeld;
        /** Seconds of AUTO left (0 or more). */
        public final double timeLeftSec;
        public final boolean finished;
        /** Why the runner stopped everything, "" while it has not. */
        public final String stoppedBy;
        /** Everything that went wrong so far, oldest first. */
        public final List<String> problems;

        Status(int stepNumber, int stepCount, String stepText, String doing, Phase phase, int pollenHeld, double timeLeftSec, boolean finished,
               String stoppedBy, List<String> problems) {
            this.stepNumber = stepNumber;
            this.stepCount = stepCount;
            this.stepText = stepText;
            this.doing = doing;
            this.phase = phase;
            this.pollenHeld = pollenHeld;
            this.timeLeftSec = timeLeftSec;
            this.finished = finished;
            this.stoppedBy = stoppedBy;
            this.problems = problems;
        }

        /** One line for the screen: "step 3/7 SHOOT 4 ...: waiting for the camera ... | POLLEN 4 | 21.3 s left". */
        public String line() {
            return String.format(Locale.ROOT, "step %d/%d  %s: %s  |  POLLEN %d  |  %.1f s left", stepNumber, stepCount, stepText, doing, pollenHeld, timeLeftSec);
        }
    }

    /** One thing that happened, with the route time and the step it belongs to (1-based, 0 for the whole run). */
    public static final class Event {
        public final double timeSec;
        public final int stepNumber;
        public final String text;
        /** True if it is a problem (a move that never arrived, a volley that faulted, ...), false for a plain happening. */
        public final boolean failure;

        Event(double timeSec, int stepNumber, String text, boolean failure) {
            this.timeSec = timeSec;
            this.stepNumber = stepNumber;
            this.text = text;
            this.failure = failure;
        }

        @Override
        public String toString() {
            return String.format(Locale.ROOT, "%.2f s, step %d: %s%s", timeSec, stepNumber, failure ? "PROBLEM: " : "", text);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------------

    private final RouteAnalysis plan;
    private final AutoRoute route;
    private final Alliance alliance;
    private final Config cfg;
    private final FieldModel field;
    private final ShotTable table;
    private final ClusterAnalyzer analyzer;
    private final ScoringPlanner planner;
    private final HiveTracker tracker;

    private final List<Event> events = new ArrayList<Event>();
    private final List<String> failures = new ArrayList<String>();

    // what this loop measured
    private Inputs cur;
    private Pose2 pose;
    private double now;
    private VisionFrame frame = VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, 0.0);
    private List<ClusterView> views = Collections.emptyList();

    // where the run stands
    private boolean started;
    private int index;
    private Runner runner;
    private Pose2 holdPose;
    private int held;
    private boolean finished;
    private String stoppedBy = "";
    private Pose2 stopPose;
    private Phase phase = Phase.START;
    private String doing = "starting";
    /** True once a wait for a tip gave up (or a volley's confirmation did): every volley that depends on a tip is then skipped. */
    private boolean tipFailed;
    /** The cell a WAIT_FOR_TIP has just seen come up (null if none): the volley after it needs no second confirmation while the camera still agrees. */
    private Side waitConfirmedSide;

    /**
     * @param plan the played-through route IN THE FRAME OF THE ALLIANCE IT RUNS FOR ({@code route.forAlliance(alliance).analyze()}); the
     *             analysis gives the poses of the FLOWER pick-ups (and the wheel chosen), which volleys depend on a tip, and the give-up deadlines
     */
    public RouteExecutor(RouteAnalysis plan) {
        this(plan, Config.fromConstants());
    }

    public RouteExecutor(RouteAnalysis plan, Config cfg) {
        this.plan = plan;
        this.route = plan.route();
        this.alliance = route.alliance();
        this.cfg = cfg;
        this.field = plan.field().model();
        this.table = ShotTable.fromConstants();
        this.analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());
        this.planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(table));
        this.tracker = new HiveTracker(field, alliance, HiveTracker.Config.fromConstants());
        this.held = BioBuzzConstants.Auto.PRELOAD_POLLEN;
        this.holdPose = route.startPose();
        this.stopPose = route.startPose();
    }

    // ---- what the caller can ask --------------------------------------------------------------------------------------

    /** The alliance this run is for. */
    public Alliance alliance() {
        return alliance;
    }

    /** The route being run (in the alliance's frame). */
    public AutoRoute route() {
        return route;
    }

    /** The robot's belief about which HIVE cell is up (the runner updates it every loop from the camera). */
    public HiveTracker tracker() {
        return tracker;
    }

    /** POLLEN the runner believes the robot holds. */
    public int pollenHeld() {
        return held;
    }

    /** The step being run, 0-based (the route's size once the route is over). */
    public int stepIndex() {
        return index;
    }

    public Phase phase() {
        return phase;
    }

    /** True once the route is over: every step done, or stopped by the clock. */
    public boolean finished() {
        return finished;
    }

    /** Why the runner stopped everything ("" if it did not). */
    public String stoppedBy() {
        return stoppedBy;
    }

    /** Everything that went wrong, oldest first. */
    public List<String> failures() {
        return Collections.unmodifiableList(failures);
    }

    /** Everything that happened, oldest first (problems included). */
    public List<Event> events() {
        return Collections.unmodifiableList(events);
    }

    /** True once a WAIT_FOR_TIP gave up (the tip was never confirmed): every volley that depends on it is skipped from then on. */
    public boolean tipNeverConfirmed() {
        return tipFailed;
    }

    /**
     * Remembers where the robot is, which alliance it is on and what it believes about the HIVE, so the TeleOp can carry on from here
     * ({@link PoseStore}). Call it when the route is over and again when the OpMode stops.
     */
    public void saveForTeleOp(Pose2 poseNow, long nowNanos) {
        PoseStore.save(poseNow, alliance, tracker.upSide(), tracker.confirmed(), nowNanos);
    }

    // ---- the loop ------------------------------------------------------------------------------------------------------

    /** Runs one loop: call it exactly once per OpMode loop, from the first loop after PLAY. */
    public Outputs update(Inputs in) {
        Outputs out = new Outputs();
        out.drive = DriveCommand.hold(holdPose);
        out.flywheelTps = cfg.idleFlywheelTps;
        cur = in;
        now = in.nowSec;

        if (!started) {
            // START: the first loop only tells the odometry where the robot stands; what it measures in this loop is not trusted yet.
            started = true;
            out.setPose = route.startPose();
            holdPose = route.startPose();
            out.drive = DriveCommand.hold(holdPose);
            pose = route.startPose();
            index = 0;
            phase = Phase.START;
            doing = "setting the odometry to the start pose";
            event(1, "START: the odometry is set to the start pose " + route.startPose());
            Outputs first = finish(out);
            index = 1;                // the next loop starts the step after START
            return first;
        }
        if (in.pose == null) {
            stopEverything("there is no odometry pose");
            return finish(out);
        }
        pose = in.pose;
        frame = in.frame == null ? VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, in.nowSec) : in.frame;
        views = analyzer.analyze(alliance, pose.heading, frame.tags);
        tracker.update(now, frame, views, HiveTracker.sideOf(pose, field));

        if (!finished && stoppedBy.isEmpty() && now >= cfg.clockStopSec) {
            stopEverything(String.format(Locale.ROOT, "the AUTO clock reached %.1f s", cfg.clockStopSec));
        }
        if (!stoppedBy.isEmpty()) {
            out.drive = DriveCommand.hold(stopPose);
            return finish(out);
        }

        int guard = plan.steps().size() + 1;
        while (index < route.size() && guard-- > 0) {
            if (runner == null) {
                runner = begin(index);
            }
            if (!runner.tick(out)) {
                break;
            }
            runner = null;
            index++;
        }
        if (index >= route.size() && runner == null) {
            if (!finished) {
                finished = true;
                phase = Phase.FINISHED;
                doing = "the route is over: holding the last pose";
                event(index, "the route is over after " + String.format(Locale.ROOT, "%.1f", now) + " s");
            }
            out.drive = DriveCommand.hold(holdPose);
            out.flywheelTps = cfg.idleFlywheelTps;
            out.feederOn = false;
            out.intakeOn = false;
        }
        return finish(out);
    }

    private void stopEverything(String why) {
        stoppedBy = why;
        finished = true;
        phase = Phase.STOPPED;
        doing = "stopped: " + why;
        stopPose = pose != null ? pose : holdPose;
        runner = null;
        failures.add("stopped: " + why);
        events.add(new Event(now, Math.min(index, route.size()), "stopped everything: " + why, false));
    }

    private Outputs finish(Outputs out) {
        out.finished = finished;
        if (!stoppedBy.isEmpty()) {
            out.drive = DriveCommand.hold(stopPose);
            out.flywheelTps = cfg.idleFlywheelTps;
            out.feederOn = false;
            out.intakeOn = false;
        }
        int shown = Math.min(index, route.size() - 1);
        String text = route.step(shown).describe();
        out.status = new Status(Math.min(index, route.size()) + (index < route.size() ? 1 : 0), route.size(), text, doing, phase, held,
                Math.max(0.0, cfg.clockStopSec - now), finished, stoppedBy, Collections.unmodifiableList(new ArrayList<String>(failures)));
        return out;
    }

    private void event(int stepNumber, String text) {
        events.add(new Event(now, stepNumber, text, false));
    }

    private void fail(String text) {
        String what = "step " + (index + 1) + ": " + text;
        failures.add(what);
        events.add(new Event(now, index + 1, text, true));
    }

    private Pose2 here() {
        return new Pose2(pose.x, pose.y, pose.heading);
    }

    // ---- the steps ------------------------------------------------------------------------------------------------------

    /** One step of the route while it runs. */
    private interface Runner {
        /** Runs one loop of the step and returns true when the step is over (finished, skipped or given up). */
        boolean tick(Outputs out);
    }

    private Runner begin(int i) {
        RouteAnalysis.StepInfo info = plan.step(i);
        Step step = route.step(i);
        switch (step.kind()) {
            case DRIVE:
                return new DriveRunner((Step.Drive) step, info);
            case INTAKE: {
                Step.Intake in = (Step.Intake) step;
                return in.source() == IntakeSource.FLOWER ? new FlowerRunner(in, info) : new IntakeRunner(in);
            }
            case SHOOT:
                return new ShootRunner((Step.Shoot) step, info);
            case WAIT:
                return new WaitRunner((Step.Wait) step);
            case WAIT_FOR_TIP:
                return new TipWaitRunner(info);
            case PARK:
                return new ParkRunner((Step.Park) step);
            default:
                throw new IllegalStateException("unexpected step " + step.kind() + " at step " + (i + 1));
        }
    }

    // ---- arrival and the straight move ------------------------------------------------------------------------------

    /** Decides when a move is finished: within the tolerance and slow, or stopped close by for a moment. */
    private final class Arrival {
        private final double posTolIn;
        private final double headingTolRad;
        private final double speedTolInS;
        private double stillSince = Double.NaN;

        /** @param headingTolDeg how well the heading must match; use {@link Double#POSITIVE_INFINITY} to ignore it */
        Arrival(double posTolIn, double headingTolDeg, double speedTolInS) {
            this.posTolIn = posTolIn;
            this.headingTolRad = Math.toRadians(headingTolDeg);
            this.speedTolInS = speedTolInS;
        }

        boolean arrived(Pose2 target) {
            double d = pose.distanceTo(target);
            double h = Angles.distance(pose.heading, target.heading);
            boolean slow = cur.speedInS <= speedTolInS;
            if (slow && d <= posTolIn && h <= headingTolRad) {
                return true;
            }
            if (slow && d <= cfg.arriveLooseIn && h <= headingTolRad) {
                if (Double.isNaN(stillSince)) {
                    stillSince = now;
                }
                return now - stillSince >= cfg.arriveLooseStillSec;
            }
            stillSince = Double.NaN;
            return false;
        }
    }

    /** A straight move to a pose (or any prepared command): holds the command until it has arrived or its time is up. */
    private final class Mover {
        private final Pose2 target;
        private final DriveCommand command;
        private final Arrival arrival;
        private final double timeoutSec;
        private final double startSec;
        private boolean timedOut;

        Mover(Pose2 target, DriveCommand command, Arrival arrival, double headingTolDeg) {
            this.target = target;
            this.command = command;
            this.arrival = arrival;
            this.startSec = now;
            double turnRad = Angles.distance(pose.heading, target.heading);
            this.timeoutSec = pose.distanceTo(target) / cfg.moveTimeoutSpeedInS + cfg.moveTimeoutExtraSec
                    + (turnRad > Math.toRadians(headingTolDeg) ? cfg.turnTimeoutSec : 0.0);
        }

        /** The straight move to {@code target}: through one waypoint, turning evenly. */
        Mover(Pose2 target, double posTolIn, double headingTolDeg, double speedTolInS) {
            this(target, DriveCommand.waypoints(Collections.singletonList(new double[] {target.x, target.y}), target, 0.0, 0.0),
                    new Arrival(posTolIn, headingTolDeg, speedTolInS), headingTolDeg);
        }

        /** Returns true when the move is over (arrived, or given up: see {@link #timedOut()}). */
        boolean tick(Outputs out) {
            out.drive = command;
            if (arrival.arrived(target)) {
                return true;
            }
            if (now - startSec > timeoutSec) {
                timedOut = true;
                return true;
            }
            return false;
        }

        boolean timedOut() {
            return timedOut;
        }
    }

    // ---- DRIVE -----------------------------------------------------------------------------------------------------------

    /**
     * Follows one {@link DrivePath}: optionally a turn on the spot first (what the checker swept), then the leg. Used by the DRIVE step
     * (the path is built from where the robot stands) and by the straight approach and retreat of a FLOWER pick-up (the paths of the plan).
     */
    private final class LegDrive {
        private final Step.Drive step;
        private final double speedFraction;
        private final double posTolIn;
        private final Arrival arrival;
        private DrivePath leg;
        private boolean turning;
        private Pose2 turnHold;
        private double turnStartSec;
        private double legStartSec;
        private double legTimeoutSec;
        private boolean timedOut;
        private boolean turnTimedOut;

        /** A DRIVE step: the leg is built from the measured pose, after a turn on the spot if the leg starts with a big one. */
        LegDrive(Step.Drive step) {
            this.step = step;
            this.speedFraction = step.speedFraction();
            this.posTolIn = cfg.arrivePosTolIn;
            this.arrival = new Arrival(posTolIn, Double.POSITIVE_INFINITY, cfg.arriveSpeedInS);
            DrivePath first = DrivePath.of(here(), step);
            if (Math.abs(first.preTurnSignedRadians()) > Math.toRadians(cfg.preTurnMinDeg)) {
                turning = true;
                turnHold = new Pose2(pose.x, pose.y, first.headingAt(0.0));
                turnStartSec = now;
            } else {
                startLeg(first);
            }
        }

        /** A ready-made path (the straight approach or retreat of a FLOWER pick-up) at a fraction of the cruise speed. */
        LegDrive(DrivePath path, double speedFraction, double posTolIn) {
            this.step = null;
            this.speedFraction = speedFraction;
            this.posTolIn = posTolIn;
            this.arrival = new Arrival(posTolIn, Double.POSITIVE_INFINITY, cfg.arriveSpeedInS);
            startLeg(path);
        }

        private void startLeg(DrivePath path) {
            leg = path;
            legStartSec = now;
            double turnAllowance = Math.toDegrees(path.totalTurnRadians()) > cfg.turnDoneDeg ? cfg.turnTimeoutSec : 0.0;
            legTimeoutSec = path.length() / (cfg.moveTimeoutSpeedInS * speedFraction) + cfg.moveTimeoutExtraSec + turnAllowance;
        }

        boolean turning() {
            return turning;
        }

        boolean timedOut() {
            return timedOut;
        }

        boolean turnTimedOut() {
            return turnTimedOut;
        }

        DrivePath leg() {
            return leg;
        }

        /** Returns true when the leg is over (arrived, or given up). */
        boolean tick(Outputs out) {
            if (turning) {
                out.drive = DriveCommand.hold(turnHold);
                boolean there = Angles.distance(pose.heading, turnHold.heading) <= Math.toRadians(cfg.turnDoneDeg);
                if (there || now - turnStartSec > cfg.turnTimeoutSec) {
                    turnTimedOut = !there;
                    turning = false;
                    startLeg(DrivePath.of(here(), step));      // from where the turn ended
                } else {
                    return false;
                }
            }
            out.drive = DriveCommand.leg(leg, speedFraction);
            if (arrival.arrived(leg.end())) {
                return true;
            }
            if (now - legStartSec > legTimeoutSec) {
                timedOut = true;
                return true;
            }
            return false;
        }
    }

    private final class DriveRunner implements Runner {
        private final Step.Drive step;
        private final LegDrive drive;

        DriveRunner(Step.Drive step, RouteAnalysis.StepInfo info) {
            this.step = step;
            this.drive = new LegDrive(step);
            phase = drive.turning() ? Phase.TURNING : Phase.DRIVING;
            doing = describe();
        }

        private String describe() {
            return drive.turning() ? "turning to the heading the leg starts with" : "following " + step.describe();
        }

        @Override
        public boolean tick(Outputs out) {
            boolean done = drive.tick(out);
            phase = drive.turning() ? Phase.TURNING : Phase.DRIVING;
            doing = describe();
            if (!done) {
                return false;
            }
            if (drive.turnTimedOut()) {
                fail("the turn on the spot before the leg did not reach its heading in " + cfg.turnTimeoutSec + " s; went on anyway");
            }
            if (drive.timedOut()) {
                fail("the leg " + step.describe() + " did not arrive in time: given up, the route goes on from where the robot is");
                holdPose = here();
            } else {
                holdPose = drive.leg().end();
            }
            return true;
        }
    }

    // ---- WAIT, WAIT_FOR_TIP, PARK ----------------------------------------------------------------------------------------

    private final class WaitRunner implements Runner {
        private final double untilSec;

        WaitRunner(Step.Wait step) {
            this.untilSec = now + step.seconds();
            phase = Phase.WAITING;
            doing = String.format(Locale.ROOT, "waiting %.1f s", step.seconds());
        }

        @Override
        public boolean tick(Outputs out) {
            out.drive = DriveCommand.hold(holdPose);
            return now >= untilSec;
        }
    }

    private final class TipWaitRunner implements Runner {
        private final Side target;
        private final TipWait wait;

        TipWaitRunner(RouteAnalysis.StepInfo info) {
            Side side = null;
            for (int j = info.index + 1; j < plan.steps().size(); j++) {
                RouteAnalysis.StepInfo next = plan.step(j);
                if (next.step instanceof Step.Shoot) {
                    side = next.targetSide;
                    break;
                }
            }
            this.target = side;
            if (side == null) {
                this.wait = null;
                return;
            }
            double deadline = plan.effectiveGiveUpSec(info);
            this.wait = new TipWait(new TipConfirmation(tracker, side, TipConfirmation.Config.fromConstants()), deadline);
            phase = Phase.WAITING_FOR_TIP;
            doing = String.format(Locale.ROOT, "waiting for the camera to see the %s cell tipped up (gives up at %.1f s)", side, deadline);
        }

        @Override
        public boolean tick(Outputs out) {
            if (wait == null) {
                return true;             // no volley after it: nothing to wait for
            }
            out.drive = DriveCommand.hold(holdPose);
            TipWait.Outcome outcome = wait.update(now, frame, views);
            if (outcome == TipWait.Outcome.CONFIRMED) {
                waitConfirmedSide = target;
                event(index + 1, "the camera has seen the " + target + " cell up and steady: the HIVE has tipped");
                return true;
            }
            if (outcome == TipWait.Outcome.GAVE_UP) {
                tipFailed = true;
                fail("gave up waiting for the HIVE to tip (" + wait.why(now) + "): every volley that needs the tip is skipped, the route goes on to LEAVE and PARK");
                return true;
            }
            doing = String.format(Locale.ROOT, "waiting for the %s cell to come up: %s", target, wait.why(now));
            return false;
        }
    }

    private final class ParkRunner implements Runner {
        private final Step.Park step;
        private final Mover mover;

        ParkRunner(Step.Park step) {
            this.step = step;
            this.mover = new Mover(step.pose(), cfg.arrivePosTolIn, cfg.turnDoneDeg, cfg.arriveSpeedInS);
            phase = Phase.PARKING;
            doing = "driving to the park pose " + step.pose();
        }

        @Override
        public boolean tick(Outputs out) {
            if (!mover.tick(out)) {
                return false;
            }
            if (mover.timedOut()) {
                fail("did not reach the park pose in time: it parks where it is");
                holdPose = here();
            } else {
                holdPose = step.pose();
            }
            return true;
        }
    }

    // ---- INTAKE: GARDEN and FLOOR ---------------------------------------------------------------------------------------

    private final class IntakeRunner implements Runner {
        private final Step.Intake step;
        private final Mover mover;
        private final int take;
        private boolean dwelling;
        private double dwellStartSec;
        private double stopAfterSec;

        IntakeRunner(Step.Intake step) {
            this.step = step;
            int room = Math.max(0, cfg.capacity - held);
            this.take = Math.min(step.expectedCount(), room);
            this.mover = take > 0 ? new Mover(step.robotPose(), cfg.arrivePosTolIn, cfg.turnDoneDeg, cfg.arriveSpeedInS) : null;
            phase = Phase.MOVING;
            doing = "driving to the " + step.source() + " pick-up pose";
        }

        @Override
        public boolean tick(Outputs out) {
            if (take <= 0) {
                fail("the robot already holds " + held + " POLLEN: nothing is taken (G407, at most " + cfg.capacity + ")");
                return true;
            }
            if (!dwelling) {
                if (!mover.tick(out)) {
                    return false;
                }
                if (mover.timedOut()) {
                    fail("did not reach the " + step.source() + " pick-up pose in time: it takes POLLEN where it is");
                    holdPose = here();
                } else {
                    holdPose = step.robotPose();
                }
                dwelling = true;
                dwellStartSec = now;
                // The POLLEN come in at an even rate over the dwell, so the intake is switched off when the last one that fits is in (G407).
                stopAfterSec = step.dwellSec() * take / step.expectedCount();
            }
            double elapsed = now - dwellStartSec;
            phase = Phase.INTAKING;
            doing = String.format(Locale.ROOT, "taking POLLEN from the %s (%d to take)", step.source(), take);
            out.drive = DriveCommand.hold(holdPose);
            if (elapsed >= stopAfterSec) {
                held += take;
                event(index + 1, "took " + take + " POLLEN from the " + step.source() + ": holds " + held);
                return true;
            }
            out.intakeOn = true;
            return false;
        }
    }

    // ---- INTAKE: a FLOWER (the wiggle) -------------------------------------------------------------------------------------

    private final class FlowerRunner implements Runner {
        private final FlowerPress press;
        private Mover lineUp;
        private LegDrive approach;
        private FlowerWiggleSequence seq;
        private LegDrive retreat;
        private int stage;                // 0 line up, 1 approach, 2 wiggle, 3 retreat

        FlowerRunner(Step.Intake step, RouteAnalysis.StepInfo info) {
            FlowerPress p = info.press;
            if (p == null) {
                fail("no FLOWER is near the target point of this INTAKE: nothing is taken");
            } else if (!p.reaches()) {
                fail("the wheels do not reach the POLLEN (the validator says so): this pick-up is skipped");
                p = null;
            } else if (held >= cfg.capacity) {
                fail("the robot already holds " + held + " POLLEN: the FLOWER pick-up is skipped (G407)");
                p = null;
            }
            this.press = p;
            if (press != null) {
                lineUp = new Mover(press.lineUp, cfg.flowerLineUpTolIn, cfg.turnDoneDeg, cfg.arriveSpeedInS);
                phase = Phase.FLOWER_LINE_UP;
                doing = "lining up at the FLOWER with the " + press.wheel + " wheel";
            }
        }

        @Override
        public boolean tick(Outputs out) {
            if (press == null) {
                return true;
            }
            switch (stage) {
                case 0:
                    if (!lineUp.tick(out)) {
                        return false;
                    }
                    if (lineUp.timedOut()) {
                        fail("could not line up at the FLOWER in time: the pick-up is skipped");
                        holdPose = here();
                        return true;
                    }
                    approach = new LegDrive(press.approach, cfg.flowerSpeedFraction, cfg.flowerStartTolIn);
                    stage = 1;
                    phase = Phase.FLOWER_APPROACH;
                    doing = "driving straight in, slowly";
                    return tick(out);
                case 1:
                    if (!approach.tick(out)) {
                        return false;
                    }
                    if (approach.timedOut()) {
                        fail("the straight approach to the FLOWER did not arrive in time: backing out, the pick-up is skipped");
                        startRetreat();
                        return tick(out);
                    }
                    seq = new FlowerWiggleSequence(press, held, now);
                    stage = 2;
                    phase = Phase.FLOWER_WIGGLE;
                    return tick(out);
                case 2: {
                    FlowerWiggleSequence.Command c = seq.update(now, pose.heading);
                    if (seq.finished()) {
                        startRetreat();
                        return tick(out);
                    }
                    out.drive = DriveCommand.hold(c.hold);
                    out.intakeOn = c.intakeOn;
                    doing = String.format(Locale.ROOT, "wiggling the %s wheel into the POLLEN (%d cycles done, %d POLLEN out)", press.wheel, seq.cyclesDone(),
                            seq.pollenTaken());
                    return false;
                }
                default:
                    if (!retreat.tick(out)) {
                        return false;
                    }
                    if (retreat.timedOut()) {
                        fail("the straight retreat from the FLOWER did not arrive in time");
                        holdPose = here();
                    } else {
                        holdPose = press.retreat;
                    }
                    if (seq != null) {
                        if (seq.failed()) {
                            fail("a turn of the wiggle never reached its heading: the rest of this pick-up was skipped after "
                                    + seq.cyclesDone() + " cycles");
                        }
                        held = Math.min(cfg.capacity, held + seq.pollenTaken());
                        event(index + 1, "FLOWER pick-up over: " + seq.pollenTaken() + " POLLEN taken, the robot holds " + held);
                    }
                    return true;
            }
        }

        private void startRetreat() {
            retreat = new LegDrive(press.retreatPath, cfg.flowerSpeedFraction, cfg.flowerStartTolIn);
            stage = 3;
            phase = Phase.FLOWER_RETREAT;
            doing = "backing straight out, slowly";
        }
    }

    // ---- SHOOT -----------------------------------------------------------------------------------------------------------

    private enum Stage { MOVE, CONFIRM, FIRE }

    private final class ShootRunner implements Runner {
        private final Step.Shoot step;
        private final RouteAnalysis.StepInfo info;
        private final Side target;
        private final double startSec;
        private final int volley;
        private String skipReason = "";
        private Stage stage = Stage.FIRE;
        private Mover mover;
        /** Where the robot is to stand for the volley when the step moves it there (the pose of the step, or the one the planner chose). */
        private Pose2 standPose;
        private boolean confirmNeeded;
        private TipWait wait;
        private AutoShootController controller;
        private boolean primed;

        ShootRunner(Step.Shoot step, RouteAnalysis.StepInfo info) {
            this.step = step;
            this.info = info;
            this.target = info.targetSide;
            this.startSec = now;
            this.volley = Math.min(step.count(), held);
            phase = Phase.SHOOTING;
            if (volley <= 0) {
                skipReason = "the robot holds no POLLEN, so there is nothing to shoot";
            } else if (info.tipDependent) {
                if (tipFailed) {
                    skipReason = "the HIVE tip this volley needs was never confirmed by the camera (no camera, no shot)";
                } else if (!frame.usable()) {
                    skipReason = "the camera is not working (" + frame.status + "), so the tip this volley needs cannot be confirmed (no camera, no shot)";
                    tipFailed = true;
                }
            } else if (tracker.upSide() != target) {
                skipReason = "the robot believes the " + tracker.upSide() + " cell is up now, not the " + target + " cell the route shoots at: not firing at a cell that may face down (G417)";
            }
            if (!skipReason.isEmpty()) {
                return;
            }
            if (step.poseOrNull() != null) {
                Pose2 p = step.poseOrNull();
                mover = new Mover(p, cfg.handoffIn, cfg.handoffDeg, cfg.handoffSpeedInS);
                standPose = p;
                stage = Stage.MOVE;
                doing = "driving to the shooting pose " + p;
            } else if (info.tipDependent) {
                // The cell that comes up is on the far side of the HIVE: drive there by the planner's checked route, and wait there for the camera.
                ScoringPlanner.Solution sol = planner.solve(pose, field.cell(alliance, target));
                if (!sol.ok()) {
                    skipReason = "the planner found no place to shoot the " + target + " cell from";
                    return;
                }
                DriveCommand cmd = DriveCommand.waypoints(sol.waypoints, sol.pose, sol.turnHoldIn, sol.turnSettleIn);
                mover = new Mover(sol.pose, cmd, new Arrival(cfg.handoffIn, cfg.handoffDeg, cfg.handoffSpeedInS), cfg.handoffDeg);
                standPose = sol.pose;
                stage = Stage.MOVE;
                doing = "driving to the " + target + " cell";
            } else {
                doing = "aiming";
            }
            confirmNeeded = info.tipDependent && !(waitConfirmedSide == target && tracker.upSide() == target);
        }

        @Override
        public boolean tick(Outputs out) {
            if (!skipReason.isEmpty()) {
                event(index + 1, "volley skipped: " + skipReason);
                if (info.tipDependent) {
                    // a skipped volley that needed the tip is a problem worth showing (a volley with nothing left to shoot is not)
                    failures.add("step " + (index + 1) + ": volley skipped: " + skipReason);
                }
                holdPose = here();
                return true;
            }
            if (now - startSec > cfg.shootStepTimeoutSec) {
                fail("the shoot step took longer than " + cfg.shootStepTimeoutSec + " s: given up");
                return endVolley(controller == null ? 0 : controller.shotsFired());
            }
            if (stage == Stage.MOVE) {
                if (!mover.tick(out)) {
                    return false;
                }
                if (mover.timedOut()) {
                    fail("did not reach the shooting pose in time: the shooting sequence plans from where the robot is");
                    holdPose = here();
                } else {
                    holdPose = standPose;       // wait here for the camera: the pose the move was for, not the one the robot came from
                }
                stage = confirmNeeded ? Stage.CONFIRM : Stage.FIRE;
                if (stage == Stage.CONFIRM) {
                    startConfirm();
                } else {
                    doing = "aiming";
                }
            }
            if (stage == Stage.CONFIRM) {
                if (wait == null) {
                    startConfirm();
                }
                out.drive = DriveCommand.hold(holdPose);
                TipWait.Outcome outcome = wait.update(now, frame, views);
                if (outcome == TipWait.Outcome.WAITING) {
                    phase = Phase.CONFIRMING_TIP;
                    doing = String.format(Locale.ROOT, "waiting for the camera to see the %s cell up and steady: %s", target, wait.why(now));
                    return false;
                }
                if (outcome == TipWait.Outcome.GAVE_UP) {
                    tipFailed = true;
                    fail("the camera never confirmed the " + target + " cell up in time (" + wait.why(now) + "): the volley is skipped");
                    holdPose = here();
                    return true;
                }
                event(index + 1, "the camera has seen the " + target + " cell up and steady");
                stage = Stage.FIRE;
            }
            return fire(out);
        }

        private void startConfirm() {
            // The latest moment the camera may confirm and the volley and the rest of the route still fit: the plan's launch time moved to the budget.
            double deadline = cfg.timeBudgetSec - plan.totalSeconds() + info.launchSec;
            wait = new TipWait(new TipConfirmation(tracker, target, TipConfirmation.Config.fromConstants()), deadline);
            phase = Phase.CONFIRMING_TIP;
            doing = "waiting for the camera to see the " + target + " cell up and steady";
            stage = Stage.CONFIRM;
        }

        private boolean fire(Outputs out) {
            phase = Phase.SHOOTING;
            if (controller == null) {
                AutoShootController.Config c = AutoShootController.Config.fromConstants();
                c.maxShots = volley;
                controller = new AutoShootController(alliance, field, planner, table, tracker, c);
            }
            AutoShootController.Inputs ci = new AutoShootController.Inputs();
            ci.nowSec = now;
            ci.shootHeld = primed;       // the controller must see the button up once before a press
            ci.pose = pose;
            ci.speedInS = cur.speedInS;
            ci.omegaRadS = cur.omegaRadS;
            ci.poseTrusted = true;
            ci.flywheelTps = cur.flywheelTps;
            ci.frame = frame;
            ci.views = views;
            ci.stickMagnitude = 0.0;
            AutoShootController.Outputs co = controller.update(ci);
            if (!primed) {
                primed = true;
                out.drive = DriveCommand.hold(holdPose);
                return false;
            }
            if (info.tipDependent && !frame.usable()) {
                fail("the camera stopped working during a volley that depends on a tip: the volley is stopped (no camera, no shot)");
                return endVolley(co.shotsFired);
            }
            if (co.targetSide != null && co.targetSide != target) {
                event(index + 1, "the camera saw the HIVE tip: the volley ends after " + co.shotsFired + " shot(s) (the other cell is up now)");
                return endVolley(co.shotsFired);
            }
            if (co.state == AutoShootController.State.FAULT) {
                fail("the shooting sequence stopped: " + co.fault);
                return endVolley(co.shotsFired);
            }
            if (co.state == AutoShootController.State.DONE) {
                event(index + 1, "volley done: " + co.shotsFired + " shot(s)");
                return endVolley(co.shotsFired);
            }
            doing = co.message + (!Double.isNaN(co.rangeIn) ? String.format(Locale.ROOT, " (%d/%d shots, range %.1f in)", co.shotsFired, volley, co.rangeIn) : "");
            switch (co.driveMode) {
                case FOLLOW:
                    out.drive = DriveCommand.waypoints(co.waypoints, co.holdPose, co.turnHoldIn, co.turnSettleIn);
                    break;
                case HOLD:
                    out.drive = DriveCommand.hold(co.holdPose);
                    break;
                default:
                    out.drive = DriveCommand.hold(holdPose);
                    break;
            }
            out.flywheelTps = co.flywheelTargetTps;
            out.feederOn = co.feederOn;
            return false;
        }

        private boolean endVolley(int shots) {
            held = Math.max(0, held - shots);
            if (waitConfirmedSide != null && tracker.upSide() != waitConfirmedSide) {
                waitConfirmedSide = null;
            }
            holdPose = here();
            return true;
        }
    }
}
