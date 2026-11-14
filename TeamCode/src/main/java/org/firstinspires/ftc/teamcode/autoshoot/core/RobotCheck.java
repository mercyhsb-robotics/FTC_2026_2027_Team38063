package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheckText.f0;
import static org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheckText.f1;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheckJudge.Judgement;

/**
 * The guided robot check ("00 Robot Check"): a step-by-step walk through every important part of the robot, written for
 * a team that has never run it. This class is the whole brain of it; the OpMode only reads the hardware, hands the
 * readings to {@link #update}, shows the text it returns and applies the motor commands it returns.
 *
 * <p>The steps, in order (numbered like the screens): 0 safety, 1 battery, 2 hardware names, 3 drive wheels, 4 Pinpoint
 * odometry, 5 intake, 6 flywheel, 7 feeder, 8 Limelight, 9 gamepad, 10 summary. Every step follows the same pattern:
 * <ol>
 *   <li><b>Ready screen:</b> what you do, what the robot will do, how to continue. Nothing moves yet. The yes button
 *       starts it, the skip button skips the step.</li>
 *   <li><b>Running:</b> a motor turns for a short, fixed time at a gentle power (the B button stops it at once).</li>
 *   <li><b>Question or waiting:</b> you answer yes / no, or you do something by hand (push the robot, hold a tag, press
 *       buttons).</li>
 *   <li><b>Result screen:</b> PASS, WARN or FAIL, what was seen, and the one thing to do. The yes button goes on, the
 *       repeat button does the step again.</li>
 * </ol>
 * The summary lists every step and stays on screen until the OpMode is stopped.
 *
 * <p><b>Fail safe.</b> A motor command is only ever produced while a step is in its running phase, and that phase can
 * only be entered by a button press. Every command is clamped to {@code BioBuzzConstants.RobotCheck.POWER_LIMIT}. The
 * moment a step leaves its running phase the next {@link Outputs} has every command at zero.
 *
 * <p><b>Words used here.</b> A <b>verdict</b> is PASS (fine), WARN (works, but look at it), FAIL (fix it) or SKIPPED.
 * An <b>edge</b> is the moment a button goes from up to down: only that moment counts as a press, so holding a button
 * never repeats it. A <b>lockout</b> is a short pause after each screen change during which presses are ignored, so a
 * double click cannot race through two screens.
 *
 * <p>Plain Java, no FTC SDK: the clock and all readings come in through {@link Inputs}, so unit tests drive it with
 * made-up numbers. Every threshold is in {@link RobotCheckConfig} (from {@code BioBuzzConstants.RobotCheck}); every
 * decision and every piece of advice is in {@link RobotCheckJudge}.
 */
public final class RobotCheck {

    /** The outcome of a step. */
    public enum Verdict {
        NOT_RUN, PASS, WARN, FAIL, SKIPPED
    }

    /** Which drive the TeleOp really uses: Pedro with the Pinpoint, or the backup MecanumDrive class. */
    public enum DriveKind {
        PEDRO, LEGACY
    }

    /** The steps, in the order they are shown. */
    public enum StepId {
        SAFETY("Safety first"),
        BATTERY("Battery"),
        HARDWARE_NAMES("Hardware names"),
        DRIVE_WHEELS("Drive wheels"),
        PINPOINT("Pinpoint odometry"),
        INTAKE("Intake"),
        FLYWHEEL("Flywheel"),
        FEEDER("Feeder"),
        LIMELIGHT("Limelight camera"),
        GAMEPAD("Gamepad"),
        SUMMARY("Summary");

        public final String title;

        StepId(String title) {
            this.title = title;
        }
    }

    /** "No wheel is being driven" in {@link Commands#wheel}. */
    public static final int NO_WHEEL = -1;

    /** Everything the OpMode measures, refreshed every loop. The logic reads only what the current step needs. */
    public static final class Inputs {
        /** The clock, in seconds, counting up from any starting point. */
        public double nowSec;
        /** The gamepad buttons held down right now. */
        public final Set<Button> buttons = EnumSet.noneOf(Button.class);
        /** The two triggers, true while pulled past their threshold. */
        public boolean leftTrigger;
        public boolean rightTrigger;
        /** The kind of controller the Driver Station reports (for display only), for example SONY_PS4. */
        public String gamepadType = "";

        /** Which devices were found in the robot configuration. Starts as "all of them". */
        public final Set<RobotCheckDevice> present = EnumSet.allOf(RobotCheckDevice.class);
        /** Which drive the TeleOp would use, and (for the backup drive) why Pedro could not start. */
        public DriveKind driveKind = DriveKind.PEDRO;
        public String driveNote = "";

        /** The battery voltage in volts, or NaN if it could not be read. */
        public double batteryVolts = Double.NaN;

        /** False if Pedro / the Pinpoint could not start; then {@link #poseProblem} says why. */
        public boolean poseAvailable;
        public String poseProblem = "";
        /** The pose: x and y in inches, heading in radians (counter-clockwise positive). */
        public double poseXIn;
        public double poseYIn;
        public double poseHeadingRad;
        /** The Pinpoint's own status name (for example READY or FAULT_X_POD_NOT_DETECTED); empty if unknown. */
        public String pinpointStatus = "";

        /** The flywheel's measured speed in ticks per second. */
        public double flywheelTps;

        /** Whether the Control Hub can reach the Limelight, how many camera pictures it has received in total, and the latest result. */
        public boolean visionConnected;
        public int visionFrames;
        /** The latest camera result; null if there is no camera object at all. */
        public VisionFrame frame;
    }

    /**
     * What the hardware must do right now. Everything is zero when no test is running. The OpMode applies ALL of these
     * every loop, so a zero here really switches that part off.
     */
    public static final class Commands {
        /** The wheel to spin (0 front-left, 1 back-left, 2 front-right, 3 back-right) or {@link #NO_WHEEL}. */
        public int wheel = NO_WHEEL;
        /** Power for that wheel, -1 to 1. Already signed for the drive in use (the backup drive drives forward with negative power). */
        public double wheelPower;
        public double intakeMotorPower;
        public double leftServoPower;
        public double rightServoPower;
        /** Flywheel target speed in ticks per second; 0 = stopped. */
        public double flywheelTps;
        /** Feeder servo power, positive = forward (toward the flywheel). */
        public double feederPower;

        /** True if any motor or servo is being told to move. */
        public boolean anyOn() {
            return wheel != NO_WHEEL || intakeMotorPower != 0.0 || leftServoPower != 0.0 || rightServoPower != 0.0
                    || flywheelTps != 0.0 || feederPower != 0.0;
        }

        private static double clamp(double v, double limit) {
            return Math.max(-limit, Math.min(limit, v));
        }

        void limit(double powerLimit) {
            wheelPower = clamp(wheelPower, powerLimit);
            intakeMotorPower = clamp(intakeMotorPower, powerLimit);
            leftServoPower = clamp(leftServoPower, powerLimit);
            rightServoPower = clamp(rightServoPower, powerLimit);
            feederPower = clamp(feederPower, powerLimit);
        }
    }

    /** What to show and what to do after one {@link #update}. */
    public static final class Outputs {
        public final StepId step;
        /** For example "Step 3 of 10: Drive wheels". */
        public final String title;
        /** The text for the Driver Station, one short plain sentence per line. */
        public final List<String> lines;
        public final Commands commands;
        /** True once the summary is showing. */
        public final boolean finished;

        Outputs(StepId step, String title, List<String> lines, Commands commands, boolean finished) {
            this.step = step;
            this.title = title;
            this.lines = Collections.unmodifiableList(lines);
            this.commands = commands;
            this.finished = finished;
        }
    }

    // ------------------------------------------------------------------------------------- state

    /** Where in a step we are. */
    private enum Phase {
        READY, RUNNING, ASKING, WAITING, RESULT
    }

    /** The button presses (edges) of this update, after the lockout. */
    private static final class Edges {
        boolean yes;
        boolean no;
        // The no button pressed while a motor runs: it stops the motor at once, so the lockout does not apply to it.
        boolean stop;
        boolean skip;
        boolean repeat;
        boolean finish;
        boolean up;
        boolean down;
    }

    /** Where each wheel is, for the "which wheel is next" sentence (as seen from behind the robot, looking toward the front). */
    private static final String[] WHEEL_PLACE = {
            "the wheel at the FRONT on the robot's LEFT side", "the wheel at the BACK on the robot's LEFT side",
            "the wheel at the FRONT on the robot's RIGHT side", "the wheel at the BACK on the robot's RIGHT side"};

    private static final int WHEEL_COUNT = 4;
    private static final int INTAKE_PARTS = 3;

    private final RobotCheckConfig cfg;
    private final RobotCheckJudge judge;
    private final FlywheelReady flywheelReady;

    private StepId step = StepId.SAFETY;
    private Phase phase = Phase.READY;
    // Which wheel / intake part is being tested; and which part (push or turn, checking or waiting for a tag) of a step.
    private int trial;
    private int part;
    private boolean started;
    // The clock when the screen last changed (for the lockout), and when the current run / wait began.
    private double screenSince;
    private double runStart;
    private final Judgement[] results = new Judgement[StepId.values().length];
    private final boolean[] answers = new boolean[WHEEL_COUNT];
    private final boolean[] tested = new boolean[WHEEL_COUNT];
    private String notice = "";

    private Set<Button> prevButtons = EnumSet.noneOf(Button.class);
    private boolean prevLeftTrigger;
    private boolean prevRightTrigger;
    private DriveKind driveKind = DriveKind.PEDRO;
    private String driveNote = "";

    // Pinpoint step: where the pose was when the current part began, and the verdict of part 1.
    private double baseX;
    private double baseY;
    private double baseHeading;
    private Judgement pushJudgement;
    // Flywheel step.
    private double flyLast;
    private double flyPeak;
    private double flyLowest;
    // Limelight step.
    private int framesAtStart;
    private int wrongTagId = -1;
    // Gamepad step.
    private final Set<Button> seen = EnumSet.noneOf(Button.class);
    private boolean seenLeftTrigger;
    private boolean seenRightTrigger;
    // Summary.
    private int scroll;

    /** A check with the real values from BioBuzzConstants. */
    public RobotCheck() {
        this(RobotCheckConfig.fromConstants());
    }

    /** A check with the given settings (tests use this to try other numbers). */
    public RobotCheck(RobotCheckConfig cfg) {
        this.cfg = cfg;
        this.judge = new RobotCheckJudge(cfg);
        this.flywheelReady = new FlywheelReady(cfg.atSpeedBelowFrac, cfg.atSpeedAboveFrac, cfg.atSpeedFloorTps,
                cfg.atSpeedRiseSec, cfg.atSpeedFallSec);
        Arrays.fill(tested, true);
    }

    // ------------------------------------------------------------------------------- read-only answers

    /** The step now showing. */
    public StepId currentStep() {
        return step;
    }

    /** The verdict of a step, or NOT_RUN if it has no result yet. */
    public Verdict verdict(StepId id) {
        Judgement j = results[id.ordinal()];
        return j == null ? Verdict.NOT_RUN : j.verdict;
    }

    /** What was seen in a step ("" if it has no result yet). */
    public String detail(StepId id) {
        Judgement j = results[id.ordinal()];
        return j == null ? "" : j.detail;
    }

    /** The one thing to do about a step ("" for a PASS or no result yet). */
    public String advice(StepId id) {
        Judgement j = results[id.ordinal()];
        return j == null ? "" : j.advice;
    }

    /** True once the summary is showing. */
    public boolean finished() {
        return step == StepId.SUMMARY;
    }

    /** How many of the real steps (0 to 9) have this verdict. */
    public int count(Verdict v) {
        int n = 0;
        for (StepId id : StepId.values()) {
            if (id != StepId.SUMMARY && verdict(id) == v) {
                n++;
            }
        }
        return n;
    }

    /** True if the summary is showing and no step failed and none was skipped (warnings do not stop it). */
    public boolean allClear() {
        return finished() && count(Verdict.FAIL) == 0 && count(Verdict.SKIPPED) == 0 && count(Verdict.NOT_RUN) == 0;
    }

    // ----------------------------------------------------------------------------------------- update

    /** Call once per loop with the newest readings; show {@link Outputs#lines} and apply {@link Outputs#commands}. */
    public Outputs update(Inputs in) {
        double now = in.nowSec;
        if (!started) {
            started = true;
            screenSince = now;
        }
        driveKind = in.driveKind;
        driveNote = in.driveNote == null ? "" : in.driveNote;
        Edges e = readEdges(in, now);
        tick(in, e, now);
        prevButtons = EnumSet.noneOf(Button.class);
        prevButtons.addAll(in.buttons);
        prevLeftTrigger = in.leftTrigger;
        prevRightTrigger = in.rightTrigger;

        Commands c = commandsNow(now);
        c.limit(cfg.powerLimit);
        int last = StepId.SUMMARY.ordinal();
        String title = "Step " + step.ordinal() + " of " + last + ": " + step.title;
        return new Outputs(step, title, screen(in, now), c, finished());
    }

    private boolean rising(Inputs in, Button b) {
        return in.buttons.contains(b) && !prevButtons.contains(b);
    }

    private Edges readEdges(Inputs in, double now) {
        Edges e = new Edges();
        e.stop = rising(in, cfg.no);
        // Right after a screen change presses are ignored (the lockout), so a double click cannot start a motor by accident.
        if (now - screenSince < cfg.lockoutSec) {
            return e;
        }
        e.yes = rising(in, cfg.yes);
        e.no = rising(in, cfg.no);
        e.skip = rising(in, cfg.skip);
        e.repeat = rising(in, cfg.repeat);
        e.finish = rising(in, cfg.gamepadFinish);
        e.up = rising(in, cfg.scrollUp);
        e.down = rising(in, cfg.scrollDown);
        return e;
    }

    private void goTo(Phase p, double now) {
        phase = p;
        screenSince = now;
    }

    private void finish(Judgement j, double now) {
        results[step.ordinal()] = j;
        goTo(Phase.RESULT, now);
    }

    private void tick(Inputs in, Edges e, double now) {
        switch (step) {
            case SAFETY:
                tickSafety(in, e, now);
                break;
            case BATTERY:
            case HARDWARE_NAMES:
                tickInstant(in, e, now);
                break;
            case DRIVE_WHEELS:
            case INTAKE:
            case FEEDER:
                tickTrials(in, e, now);
                break;
            case PINPOINT:
                tickPinpoint(in, e, now);
                break;
            case FLYWHEEL:
                tickFlywheel(in, e, now);
                break;
            case LIMELIGHT:
                tickLimelight(in, e, now);
                break;
            case GAMEPAD:
                tickGamepad(in, e, now);
                break;
            default:
                tickSummary(e);
                break;
        }
    }

    // ------------------------------------------------------------------------------- moving between steps

    private void enterStep(Inputs in, double now, StepId next) {
        step = next;
        phase = Phase.READY;
        screenSince = now;
        trial = 0;
        part = 0;
        notice = "";
        scroll = 0;
        pushJudgement = null;
        results[next.ordinal()] = null;
        Arrays.fill(answers, false);
        Arrays.fill(tested, true);
        switch (next) {
            case DRIVE_WHEELS: {
                List<RobotCheckDevice> m = missing(in, RobotCheckDevice.WHEELS);
                if (!m.isEmpty()) {
                    finish(judge.deviceMissing(m), now);
                }
                break;
            }
            case FEEDER: {
                List<RobotCheckDevice> m = missing(in, RobotCheckDevice.FEEDER);
                if (!m.isEmpty()) {
                    finish(judge.deviceMissing(m), now);
                }
                break;
            }
            case INTAKE:
                skipMissingIntakeParts(in);
                if (trial >= INTAKE_PARTS) {
                    finish(judge.intake(answers, tested), now);
                }
                break;
            case SUMMARY:
                phase = Phase.RESULT;
                break;
            default:
                break;
        }
    }

    private void nextStep(Inputs in, double now) {
        enterStep(in, now, StepId.values()[step.ordinal() + 1]);
    }

    private void skipStep(Inputs in, double now) {
        results[step.ordinal()] = judge.skipped();
        nextStep(in, now);
    }

    private static List<RobotCheckDevice> missing(Inputs in, RobotCheckDevice... wanted) {
        List<RobotCheckDevice> out = new ArrayList<RobotCheckDevice>();
        for (RobotCheckDevice d : wanted) {
            if (!in.present.contains(d)) {
                out.add(d);
            }
        }
        return out;
    }

    /** Handles the result screen: yes goes on, repeat does the step again. */
    private void tickResult(Inputs in, Edges e, double now) {
        if (e.yes) {
            nextStep(in, now);
        } else if (e.repeat) {
            enterStep(in, now, step);
        }
    }

    // -------------------------------------------------------------------------------------- the steps

    private void tickSafety(Inputs in, Edges e, double now) {
        if (e.yes) {
            results[step.ordinal()] = judge.safetyConfirmed();
            nextStep(in, now);
        } else if (e.skip) {
            notice = "The safety step cannot be skipped.";
        } else if (e.no) {
            notice = "OK, take your time. Press " + RobotCheckText.button(cfg.yes) + " when the robot is on blocks and everyone is clear.";
        }
    }

    /** Steps with no motor and an instant answer: battery and hardware names. */
    private void tickInstant(Inputs in, Edges e, double now) {
        if (phase == Phase.RESULT) {
            tickResult(in, e, now);
            return;
        }
        if (e.skip) {
            skipStep(in, now);
        } else if (e.yes) {
            if (step == StepId.BATTERY) {
                finish(judge.battery(in.batteryVolts), now);
            } else {
                List<RobotCheckDevice> m = missing(in, RobotCheckDevice.values());
                finish(judge.names(RobotCheckDevice.values().length, m), now);
            }
        }
    }

    private int trialCount() {
        switch (step) {
            case DRIVE_WHEELS:
                return WHEEL_COUNT;
            case INTAKE:
                return INTAKE_PARTS;
            default:
                return 1;
        }
    }

    private double runSeconds() {
        switch (step) {
            case DRIVE_WHEELS:
                return cfg.driveSec;
            case INTAKE:
                return cfg.intakeSec;
            default:
                return cfg.feederForwardSec + cfg.feederReverseSec;
        }
    }

    /** Intake parts whose device is missing cannot be tested: they are marked "not tested" and skipped over. */
    private void skipMissingIntakeParts(Inputs in) {
        while (trial < INTAKE_PARTS && !in.present.contains(RobotCheckJudge.intakePartDevice(trial))) {
            tested[trial] = false;
            trial++;
        }
    }

    /** Wheels, intake parts and the feeder: ready, run, ask, then the next one. */
    private void tickTrials(Inputs in, Edges e, double now) {
        switch (phase) {
            case READY:
                if (e.skip) {
                    skipStep(in, now);
                } else if (e.yes) {
                    runStart = now;
                    goTo(Phase.RUNNING, now);
                }
                break;
            case RUNNING:
                // The no button stops the motor at once; otherwise it runs for its fixed time.
                if (e.stop || now - runStart >= runSeconds()) {
                    goTo(Phase.ASKING, now);
                }
                break;
            case ASKING:
                if (e.skip) {
                    skipStep(in, now);
                } else if (e.repeat) {
                    goTo(Phase.READY, now);
                } else if (e.yes || e.no) {
                    answers[trial] = e.yes;
                    trial++;
                    if (step == StepId.INTAKE) {
                        skipMissingIntakeParts(in);
                    }
                    if (trial >= trialCount()) {
                        finish(trialsVerdict(), now);
                    } else {
                        goTo(Phase.READY, now);
                    }
                }
                break;
            default:
                tickResult(in, e, now);
                break;
        }
    }

    private Judgement trialsVerdict() {
        switch (step) {
            case DRIVE_WHEELS:
                return judge.wheels(Arrays.copyOf(answers, WHEEL_COUNT), driveKind);
            case INTAKE:
                return judge.intake(Arrays.copyOf(answers, INTAKE_PARTS), Arrays.copyOf(tested, INTAKE_PARTS));
            default:
                return judge.feeder(answers[0]);
        }
    }

    /** The robot's own forward and left distances moved since the baseline pose, in inches. */
    private double forwardSince(Inputs in) {
        double dx = in.poseXIn - baseX;
        double dy = in.poseYIn - baseY;
        return dx * Math.cos(baseHeading) + dy * Math.sin(baseHeading);
    }

    private double leftSince(Inputs in) {
        double dx = in.poseXIn - baseX;
        double dy = in.poseYIn - baseY;
        return -dx * Math.sin(baseHeading) + dy * Math.cos(baseHeading);
    }

    private double turnedDegSince(Inputs in) {
        return Math.toDegrees(Angles.error(baseHeading, in.poseHeadingRad));
    }

    private void takeBaseline(Inputs in) {
        baseX = in.poseXIn;
        baseY = in.poseYIn;
        baseHeading = in.poseHeadingRad;
    }

    private void tickPinpoint(Inputs in, Edges e, double now) {
        switch (phase) {
            case READY:
                if (e.skip) {
                    skipStep(in, now);
                } else if (e.yes) {
                    Judgement fault = pinpointBlocked(in);
                    if (fault != null) {
                        finish(fault, now);
                    } else {
                        takeBaseline(in);
                        part = 0;
                        goTo(Phase.WAITING, now);
                    }
                }
                break;
            case WAITING:
                if (e.skip) {
                    skipStep(in, now);
                } else if (e.no) {
                    Judgement problem = judge.pinpointProblemReported(part);
                    finish(part == 0 ? problem : judge.pinpointBoth(pushJudgement, problem), now);
                } else if (e.yes) {
                    if (!in.poseAvailable) {
                        finish(judge.pinpointUnavailable(in.poseProblem), now);
                    } else if (part == 0) {
                        pushJudgement = judge.pinpointPush(forwardSince(in), leftSince(in));
                        part = 1;
                        takeBaseline(in);
                        goTo(Phase.WAITING, now);
                    } else {
                        double drift = Math.hypot(in.poseXIn - baseX, in.poseYIn - baseY);
                        Judgement turn = judge.pinpointTurn(turnedDegSince(in), drift);
                        finish(judge.pinpointBoth(pushJudgement, turn), now);
                    }
                }
                break;
            default:
                tickResult(in, e, now);
                break;
        }
    }

    /** A reason the Pinpoint step cannot even start (missing, did not start, reports a fault), or null if it can. */
    private Judgement pinpointBlocked(Inputs in) {
        List<RobotCheckDevice> m = missing(in, RobotCheckDevice.PINPOINT);
        if (!m.isEmpty()) {
            return judge.deviceMissing(m);
        }
        if (!in.poseAvailable) {
            return judge.pinpointUnavailable(in.poseProblem);
        }
        return judge.pinpointFault(in.pinpointStatus);
    }

    private void tickFlywheel(Inputs in, Edges e, double now) {
        switch (phase) {
            case READY:
                if (e.skip) {
                    skipStep(in, now);
                } else if (e.yes) {
                    // The shooter code (Launcher) starts the flywheel and the feeder together, so it needs both names.
                    List<RobotCheckDevice> m = missing(in, RobotCheckDevice.FLYWHEEL, RobotCheckDevice.FEEDER);
                    if (!m.isEmpty()) {
                        finish(judge.deviceMissing(m), now);
                    } else {
                        runStart = now;
                        flyLast = 0.0;
                        flyPeak = 0.0;
                        flyLowest = 0.0;
                        flywheelReady.reset();
                        goTo(Phase.RUNNING, now);
                    }
                }
                break;
            case RUNNING: {
                flyLast = in.flywheelTps;
                flyPeak = Math.max(flyPeak, flyLast);
                flyLowest = Math.min(flyLowest, flyLast);
                // The same "at speed" rule the TeleOp uses: inside the window for a short time, not just passing through it.
                boolean atSpeed = flywheelReady.update(now, true, cfg.flywheelTargetTps, flyLast);
                if (atSpeed) {
                    finish(judge.flywheel(true, flyLast, flyPeak, flyLowest, false), now);
                } else if (e.stop) {
                    finish(judge.flywheel(false, flyLast, flyPeak, flyLowest, true), now);
                } else if (now - runStart >= cfg.flywheelTimeoutSec) {
                    finish(judge.flywheel(false, flyLast, flyPeak, flyLowest, false), now);
                }
                break;
            }
            default:
                tickResult(in, e, now);
                break;
        }
    }

    /** The first HIVE tag in the frame, or null. */
    private TagObservation hiveTag(Inputs in) {
        if (in.frame == null) {
            return null;
        }
        for (TagObservation t : in.frame.tags) {
            if (t.id >= cfg.tagIdMin && t.id <= cfg.tagIdMax) {
                return t;
            }
        }
        return null;
    }

    private void tickLimelight(Inputs in, Edges e, double now) {
        switch (phase) {
            case READY:
                if (e.skip) {
                    skipStep(in, now);
                } else if (e.yes) {
                    if (!in.present.contains(RobotCheckDevice.LIMELIGHT)) {
                        finish(judge.limelightMissing(false), now);
                    } else {
                        framesAtStart = in.visionFrames;
                        wrongTagId = -1;
                        part = 0;
                        runStart = now;
                        goTo(Phase.WAITING, now);
                    }
                }
                break;
            case WAITING:
                if (e.skip) {
                    skipStep(in, now);
                } else if (part == 0) {
                    // First half: watch the camera for a moment. Are pictures arriving, and is it the right pipeline?
                    if (now - runStart >= cfg.limelightCheckSec) {
                        VisionFrame.Status status = in.frame == null ? VisionFrame.Status.NOT_CONNECTED : in.frame.status;
                        Judgement link = judge.limelightLink(in.visionConnected, in.visionFrames - framesAtStart, status);
                        if (link != null) {
                            finish(link, now);
                        } else {
                            part = 1;
                            runStart = now;
                        }
                    }
                } else {
                    // Second half: wait for the student to hold a HIVE tag in front of the camera.
                    TagObservation tag = hiveTag(in);
                    if (tag != null) {
                        finish(judge.limelightTag(tag), now);
                    } else {
                        noteOtherTag(in);
                        if (e.no) {
                            finish(judge.limelightNoTag(wrongTagId, true), now);
                        } else if (now - runStart >= cfg.limelightTagTimeoutSec) {
                            finish(judge.limelightNoTag(wrongTagId, false), now);
                        }
                    }
                }
                break;
            default:
                tickResult(in, e, now);
                break;
        }
    }

    /** Remembers the id of a tag that is not a HIVE tag, so the advice can name it. */
    private void noteOtherTag(Inputs in) {
        if (in.frame != null && !in.frame.tags.isEmpty()) {
            wrongTagId = in.frame.tags.get(0).id;
        }
    }

    private int gamepadTotal() {
        return cfg.gamepadButtons.length + 2;
    }

    private List<String> gamepadNotSeen() {
        List<String> out = new ArrayList<String>();
        for (Button b : cfg.gamepadButtons) {
            if (!seen.contains(b)) {
                out.add(RobotCheckText.button(b));
            }
        }
        if (!seenLeftTrigger) {
            out.add("left trigger (LT)");
        }
        if (!seenRightTrigger) {
            out.add("right trigger (RT)");
        }
        return out;
    }

    private void tickGamepad(Inputs in, Edges e, double now) {
        switch (phase) {
            case READY:
                if (e.skip) {
                    skipStep(in, now);
                } else if (e.yes) {
                    seen.clear();
                    seenLeftTrigger = false;
                    seenRightTrigger = false;
                    runStart = now;
                    goTo(Phase.WAITING, now);
                }
                break;
            case WAITING: {
                // Only NEW presses count, so the press that started this step does not count by itself. No lockout here:
                // every button, including yes / no / skip / repeat, is being tested and must not act.
                for (Button b : cfg.gamepadButtons) {
                    if (rising(in, b)) {
                        seen.add(b);
                    }
                }
                seenLeftTrigger |= in.leftTrigger && !prevLeftTrigger;
                seenRightTrigger |= in.rightTrigger && !prevRightTrigger;
                List<String> notSeen = gamepadNotSeen();
                if (notSeen.isEmpty()) {
                    finish(judge.gamepad(notSeen, gamepadTotal()), now);
                } else if (e.finish || now - runStart >= cfg.gamepadTimeoutSec) {
                    finish(judge.gamepad(notSeen, gamepadTotal()), now);
                }
                break;
            }
            default:
                tickResult(in, e, now);
                break;
        }
    }

    private void tickSummary(Edges e) {
        if (e.down) {
            scroll++;
        }
        if (e.up) {
            scroll--;
        }
    }

    // ----------------------------------------------------------------------------------------- commands

    private Commands commandsNow(double now) {
        Commands c = new Commands();
        if (phase != Phase.RUNNING) {
            return c;
        }
        switch (step) {
            case DRIVE_WHEELS:
                c.wheel = trial;
                // Pedro drives forward with positive power; the backup MecanumDrive drives forward with negative power.
                c.wheelPower = (driveKind == DriveKind.PEDRO ? 1.0 : -1.0) * cfg.drivePower;
                break;
            case INTAKE:
                if (trial == 0) {
                    c.intakeMotorPower = cfg.intakeMotorPower;
                } else if (trial == 1) {
                    c.leftServoPower = cfg.leftServoSign * cfg.intakeServoPower;
                } else {
                    c.rightServoPower = cfg.rightServoSign * cfg.intakeServoPower;
                }
                break;
            case FEEDER:
                c.feederPower = now - runStart < cfg.feederForwardSec ? cfg.feederPower : -cfg.feederPower;
                break;
            case FLYWHEEL:
                c.flywheelTps = cfg.flywheelTargetTps;
                break;
            default:
                break;
        }
        return c;
    }

    // ------------------------------------------------------------------------------------------ screens

    private List<String> screen(Inputs in, double now) {
        List<String> l = new ArrayList<String>();
        if (!notice.isEmpty()) {
            l.add(notice);
        }
        if (step == StepId.SUMMARY) {
            summary(l);
            return l;
        }
        switch (phase) {
            case READY:
                ready(l);
                break;
            case RUNNING:
                running(l, now);
                break;
            case ASKING:
                asking(l);
                break;
            case WAITING:
                waiting(l, in, now);
                break;
            default:
                result(l);
                break;
        }
        return l;
    }

    private static String b(Button button) {
        return RobotCheckText.button(button);
    }

    private String startOrSkip(String what) {
        return "Press " + b(cfg.yes) + " " + what + ", or " + b(cfg.skip) + " to skip this step.";
    }

    private String driveInUse() {
        if (driveKind == DriveKind.PEDRO) {
            return "Drive in use: Pedro with the Pinpoint (the same as the TeleOp).";
        }
        return "Drive in use: the BACKUP drive (MecanumDrive), because Pedro could not start"
                + (driveNote.isEmpty() ? "." : " (" + driveNote + ").");
    }

    private void ready(List<String> l) {
        switch (step) {
            case SAFETY:
                l.add("1. Put the robot on blocks so all 4 wheels hang in the air.");
                l.add("2. Keep hands, hair and loose clothes away from the wheels, the front rollers and the back flywheel.");
                l.add("3. Take every POLLEN (ball) out of the robot.");
                l.add("4. One person watches the robot, ready to press STOP on the Driver Station.");
                l.add("Motors only turn after you press a button, and every test is short and gentle.");
                l.add("Press " + b(cfg.yes) + " to start the check.");
                break;
            case BATTERY:
                l.add("This reads the robot battery voltage, in volts.");
                l.add("The robot will not move.");
                l.add(startOrSkip("to read it"));
                break;
            case HARDWARE_NAMES:
                l.add("This checks that every device name the code uses (BioBuzzConstants.Hardware) is in the robot"
                        + " configuration on the Driver Station.");
                l.add("The robot will not move.");
                l.add(startOrSkip("to check the names"));
                break;
            case DRIVE_WHEELS:
                if (trial == 0) {
                    l.add("FRONT is the end with the intake rollers. BACK is the end with the flywheel and the camera.");
                    l.add("The robot must be on blocks: all 4 wheels in the air.");
                    l.add(driveInUse());
                    l.add("Each wheel spins gently forward for " + f1(cfg.driveSec) + " seconds, one at a time. You say if it turned"
                            + " forward (the top of the wheel moved toward the FRONT).");
                }
                l.add("Next: the " + RobotCheckText.wheel(trial) + " wheel (" + WHEEL_PLACE[trial] + ").");
                l.add(startOrSkip("to spin it"));
                break;
            case INTAKE:
                if (trial == 0) {
                    l.add("FRONT is the end with the intake rollers.");
                    l.add("Each part spins gently for " + f1(cfg.intakeSec) + " seconds, one at a time: the front roller motor, then"
                            + " the left corner servo, then the right corner servo.");
                    l.add("You say if it pulled POLLEN (a ball) INWARD, toward the inside of the robot. Keep fingers away from the rollers.");
                }
                l.add("Next: the " + RobotCheckJudge.intakePartName(trial) + ".");
                l.add(startOrSkip("to spin it"));
                break;
            case FLYWHEEL:
                l.add("The flywheel is the fast wheel at the BACK that throws a POLLEN.");
                l.add("It spins up to " + f0(cfg.flywheelTargetTps) + " ticks per second (its speed, counted by its encoder) for at most "
                        + f1(cfg.flywheelTimeoutSec) + " seconds, then stops.");
                l.add("Keep hands, hair and loose objects away from the back of the robot, and take out any POLLEN.");
                l.add("It passes if it reaches " + f0(judge.flywheelLowerTps()) + " to " + f0(judge.flywheelUpperTps()) + " ticks per second.");
                l.add(startOrSkip("to spin it"));
                break;
            case FEEDER:
                l.add("The feeder is the servo at the BACK that pushes a POLLEN into the flywheel.");
                l.add("It turns forward (toward the flywheel) for " + f1(cfg.feederForwardSec) + " seconds, then backward for "
                        + f1(cfg.feederReverseSec) + " seconds, gently. Keep fingers away from it.");
                l.add("You say if it turned the right way.");
                l.add(startOrSkip("to run it"));
                break;
            case PINPOINT:
                l.add("The Pinpoint works out where the robot is (its pose) from two small tracking wheels and a gyro.");
                l.add("Take the robot OFF the blocks and put it on the floor with room in front of it. The motors will not move:"
                        + " you move the robot by hand.");
                l.add("Part 1: push it straight forward about " + f0(cfg.pushDistanceIn) + " inches (a ruler, or half a floor tile).");
                l.add("Part 2: turn it on the spot a quarter turn (" + f0(cfg.turnDeg) + " degrees) counter-clockwise: the front swings to the left.");
                l.add(startOrSkip("to start"));
                break;
            case LIMELIGHT:
                l.add("The Limelight is the smart camera that finds AprilTags (black-and-white squares with a number).");
                l.add("First the camera is watched for " + f1(cfg.limelightCheckSec) + " seconds: is it connected, are new pictures"
                        + " arriving, is pipeline " + cfg.pipelineIndex + " (the AprilTag settings) running?");
                l.add("Then you hold a HIVE tag (number " + cfg.tagIdMin + " to " + cfg.tagIdMax + ") flat in front of the camera,"
                        + " which faces the BACK of the robot, about an arm's length away.");
                l.add("The robot will not move.");
                l.add(startOrSkip("to start"));
                break;
            case GAMEPAD:
                l.add("This shows which gamepad buttons the robot sees, so you can check them against the labels on the controller.");
                l.add("Press every button and both triggers once. Each one is marked SEEN. When all are seen the step passes by itself.");
                l.add("To finish early press the " + b(cfg.gamepadFinish) + ". The robot will not move.");
                l.add(startOrSkip("to start"));
                break;
            default:
                break;
        }
    }

    private void running(List<String> l, double now) {
        double elapsed = now - runStart;
        String stop = "Press " + b(cfg.no) + " to stop it at once.";
        switch (step) {
            case DRIVE_WHEELS:
                l.add("Spinning the " + RobotCheckText.wheel(trial) + " wheel now (" + f1(elapsed) + " of " + f1(cfg.driveSec) + " seconds).");
                l.add("Watch it: does the top of the wheel move toward the FRONT?");
                l.add(stop);
                break;
            case INTAKE:
                l.add("Spinning the " + RobotCheckJudge.intakePartName(trial) + " now (" + f1(elapsed) + " of " + f1(cfg.intakeSec) + " seconds).");
                l.add("Watch it: does it pull inward, toward the inside of the robot?");
                l.add(stop);
                break;
            case FEEDER:
                l.add(elapsed < cfg.feederForwardSec
                        ? "The feeder is turning FORWARD (toward the flywheel)."
                        : "The feeder is turning BACKWARD.");
                l.add(stop);
                break;
            case FLYWHEEL:
                l.add("The flywheel is spinning up. Speed now: " + f0(flyLast) + " ticks per second.");
                l.add("Goal: between " + f0(judge.flywheelLowerTps()) + " and " + f0(judge.flywheelUpperTps()) + " ticks per second.");
                l.add("Time: " + f1(elapsed) + " of " + f1(cfg.flywheelTimeoutSec) + " seconds.");
                l.add(stop);
                break;
            default:
                break;
        }
    }

    private void asking(List<String> l) {
        switch (step) {
            case DRIVE_WHEELS:
                l.add("Did the " + RobotCheckText.wheel(trial) + " wheel turn FORWARD (the top of the wheel moved toward the FRONT of the robot)?");
                break;
            case INTAKE:
                l.add("Did the " + RobotCheckJudge.intakePartName(trial) + " pull INWARD, toward the inside of the robot?");
                break;
            default:
                l.add("Did the feeder turn forward (pushing toward the flywheel) and then turn back?");
                break;
        }
        l.add(b(cfg.yes) + " = yes.   " + b(cfg.no) + " = no (or you did not see it).");
        l.add(b(cfg.repeat) + " = do it again.   " + b(cfg.skip) + " = skip this step.");
    }

    private void waiting(List<String> l, Inputs in, double now) {
        switch (step) {
            case PINPOINT:
                waitingPinpoint(l, in);
                break;
            case LIMELIGHT:
                waitingLimelight(l, in, now);
                break;
            default:
                waitingGamepad(l, in, now);
                break;
        }
    }

    private void waitingPinpoint(List<String> l, Inputs in) {
        if (in.poseAvailable) {
            l.add("Pose now: x " + f1(in.poseXIn) + " inches, y " + f1(in.poseYIn) + " inches, heading "
                    + f0(Math.toDegrees(in.poseHeadingRad)) + " degrees.");
        } else {
            l.add("No pose: the Pinpoint did not start" + (in.poseProblem == null || in.poseProblem.isEmpty() ? "." : " (" + in.poseProblem + ")."));
        }
        if (in.pinpointStatus != null && !in.pinpointStatus.isEmpty()) {
            l.add("Pinpoint status: " + in.pinpointStatus + ".");
        }
        if (part == 0) {
            l.add("Part 1 of 2: push the robot straight forward about " + f0(cfg.pushDistanceIn) + " inches.");
            l.add("So far: forward " + f1(forwardSince(in)) + " inches, sideways " + f1(leftSince(in)) + " inches (left is positive).");
            l.add("Let go of the robot, then press " + b(cfg.yes) + ".");
        } else {
            l.add("Part 2 of 2: turn the robot on the spot a quarter turn (" + f0(cfg.turnDeg) + " degrees) counter-clockwise.");
            l.add("So far: turned " + f0(turnedDegSince(in)) + " degrees (counter-clockwise is positive), position moved "
                    + f1(Math.hypot(in.poseXIn - baseX, in.poseYIn - baseY)) + " inches.");
            l.add("Let go of the robot, then press " + b(cfg.yes) + ".");
        }
        l.add(b(cfg.no) + " = nothing changes / a problem.   " + b(cfg.skip) + " = skip this step.");
    }

    private void waitingLimelight(List<String> l, Inputs in, double now) {
        if (part == 0) {
            l.add("Checking the camera for " + f1(cfg.limelightCheckSec) + " seconds...");
            l.add("Connected: " + (in.visionConnected ? "yes" : "no") + ".   New pictures so far: " + (in.visionFrames - framesAtStart) + ".");
            l.add(b(cfg.skip) + " = skip this step.");
            return;
        }
        l.add("The camera is connected, sending pictures, and running pipeline " + cfg.pipelineIndex + ".");
        l.add("Now hold a HIVE tag (number " + cfg.tagIdMin + " to " + cfg.tagIdMax + ") flat in front of the camera, about an arm's length away.");
        if (in.frame != null && !in.frame.tags.isEmpty()) {
            TagObservation t = in.frame.tags.get(0);
            l.add("The camera sees tag " + t.id + ": tx " + f1(t.txDeg) + " degrees, ty " + f1(t.tyDeg) + " degrees.");
        } else {
            l.add("The camera sees no tag yet.");
        }
        l.add("Time left: " + f0(Math.max(0.0, cfg.limelightTagTimeoutSec - (now - runStart))) + " seconds.");
        l.add(b(cfg.no) + " = I cannot get it to see a tag.   " + b(cfg.skip) + " = skip this step.");
    }

    private void waitingGamepad(List<String> l, Inputs in, double now) {
        List<String> holding = new ArrayList<String>();
        for (Button x : in.buttons) {
            holding.add(b(x));
        }
        if (in.leftTrigger) {
            holding.add("left trigger (LT)");
        }
        if (in.rightTrigger) {
            holding.add("right trigger (RT)");
        }
        l.add("Pressing now: " + (holding.isEmpty() ? "nothing" : RobotCheckText.join(holding, ", ")) + ".");
        List<String> notSeen = gamepadNotSeen();
        l.add("SEEN " + (gamepadTotal() - notSeen.size()) + " of " + gamepadTotal() + ".");
        l.add("NOT YET: " + (notSeen.isEmpty() ? "none" : RobotCheckText.join(notSeen, ", ")) + ".");
        if (in.gamepadType != null && !in.gamepadType.isEmpty()) {
            l.add("The Driver Station says this controller is: " + in.gamepadType + ".");
        }
        l.add("Press the " + b(cfg.gamepadFinish) + " to finish early. Time left: "
                + f0(Math.max(0.0, cfg.gamepadTimeoutSec - (now - runStart))) + " seconds.");
    }

    private void result(List<String> l) {
        Judgement j = results[step.ordinal()];
        l.add("Result: " + j.verdict + ".");
        l.addAll(RobotCheckText.lines(j.detail));
        if (!j.advice.isEmpty()) {
            l.add("What to do: " + j.advice);
        }
        StepId next = StepId.values()[step.ordinal() + 1];
        l.add("Press " + b(cfg.yes) + " for " + (next == StepId.SUMMARY ? "the summary" : "the next step")
                + ", or " + b(cfg.repeat) + " to do this step again.");
    }

    // ------------------------------------------------------------------------------------------ summary

    private static String firstLine(String text) {
        List<String> lines = RobotCheckText.lines(text);
        return lines.isEmpty() ? "" : lines.get(0);
    }

    /** The whole summary, before it is cut into pages: counts, the verdict, then the steps that need attention, then the passes. */
    private List<String> summaryAll() {
        List<String> all = new ArrayList<String>();
        all.add("RESULT: " + count(Verdict.PASS) + " PASS, " + count(Verdict.WARN) + " WARN, " + count(Verdict.FAIL) + " FAIL, "
                + count(Verdict.SKIPPED) + " SKIPPED. This list stays until you press STOP.");
        if (count(Verdict.FAIL) > 0) {
            all.add("NOT READY: fix every FAIL below, then run this check again.");
        } else if (count(Verdict.SKIPPED) > 0 || count(Verdict.NOT_RUN) > 0) {
            all.add("NOT FULLY CHECKED: you skipped " + (count(Verdict.SKIPPED) + count(Verdict.NOT_RUN))
                    + " step(s). Run this check again without skipping before a match.");
        } else {
            all.add("All clear: you can run the TeleOp");
            if (count(Verdict.WARN) > 0) {
                all.add("Warnings are not failures, but read them first.");
            }
        }
        // Problems first (they are what the team has to act on), each followed by its one thing to do; the passes last.
        Verdict[] order = {Verdict.FAIL, Verdict.WARN, Verdict.SKIPPED, Verdict.NOT_RUN, Verdict.PASS};
        for (Verdict wanted : order) {
            for (StepId id : StepId.values()) {
                if (id == StepId.SUMMARY || verdict(id) != wanted) {
                    continue;
                }
                all.add((wanted == Verdict.NOT_RUN ? "NOT RUN" : wanted.name()) + "  " + id.title + " - " + firstLine(detail(id)));
                if (wanted != Verdict.PASS && !advice(id).isEmpty()) {
                    all.add("    To do: " + advice(id));
                }
            }
        }
        return all;
    }

    private void summary(List<String> l) {
        List<String> all = summaryAll();
        int perPage = cfg.summaryLinesPerPage;
        if (all.size() <= perPage) {
            scroll = 0;
            l.addAll(all);
            return;
        }
        // Too long for one screen: show a window of it, with one line left for the "more" hint.
        int window = perPage - 1;
        scroll = Math.max(0, Math.min(scroll, all.size() - window));
        l.addAll(all.subList(scroll, scroll + window));
        l.add("Lines " + (scroll + 1) + " to " + (scroll + window) + " of " + all.size() + ": " + b(cfg.scrollDown) + " for more, "
                + b(cfg.scrollUp) + " to go back.");
    }
}
